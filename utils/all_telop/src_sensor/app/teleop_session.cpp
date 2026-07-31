// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// teleop_session.cpp — Pika sensor -> OpenArm Cartesian teleop as a library session
// for all_telop. Keyboard p/q is owned by the orchestrator.

#include <teleop_session.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <common.hpp>
#include <controller/cartesian_mapper.hpp>
#include <controller/cartesian_types.hpp>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <controller/joint_segment_cache.hpp>
#include <ik/ik_solver.hpp>
#include <interpolator/joint_interpolator.hpp>
#include <io/pika_pose.hpp>
#include <io/pika_pose_ros.hpp>
#include <io/tcp_relative_csv_writer.hpp>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <periodic_timer_thread.hpp>
#include <pose_health_monitor.h>
#include <pose_queue.h>
#include <robot_state.hpp>
#include <safety/safety_filter.hpp>
#include <yamlloader.hpp>

#include <rclcpp/rclcpp.hpp>

namespace {

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

struct RuntimeConfig {
    std::string arm = "right";
    std::string ik = "dls";
    std::string interpolator = "linear";
    std::string can_interface = "can0";
    std::string urdf = "assets/urdf/v1.urdf";
    bool skip_gripper = false;
    bool record_tcp_csv = false;
    std::string record_tcp_csv_path = "~/pika_ros/data/014/sensor/vive.csv";
    std::string pose_topic;  // optional override; empty => derive from arm side
};

RuntimeConfig load_runtime_config(const YAML::Node& root) {
    RuntimeConfig cfg;
    if (!root["Runtime"]) {
        throw std::runtime_error("config missing top-level 'Runtime' section");
    }
    const YAML::Node n = root["Runtime"];
    if (n["arm"]) cfg.arm = n["arm"].as<std::string>();
    if (n["ik"]) cfg.ik = n["ik"].as<std::string>();
    if (n["interpolator"]) cfg.interpolator = n["interpolator"].as<std::string>();
    if (n["can_interface"]) cfg.can_interface = n["can_interface"].as<std::string>();
    if (n["urdf"]) cfg.urdf = n["urdf"].as<std::string>();
    if (n["skip_gripper"]) cfg.skip_gripper = n["skip_gripper"].as<bool>();
    if (n["record_tcp_csv"]) cfg.record_tcp_csv = n["record_tcp_csv"].as<bool>();
    if (n["record_tcp_csv_path"]) {
        cfg.record_tcp_csv_path = n["record_tcp_csv_path"].as<std::string>();
    }
    if (n["pose_topic"]) cfg.pose_topic = n["pose_topic"].as<std::string>();
    return cfg;
}

void validate_arm_side(const std::string& arm) {
    if (arm == "right") return;
    if (arm == "left") {
        throw std::runtime_error("Runtime.arm=left is not implemented yet (only 'right' is)");
    }
    if (arm == "both") {
        throw std::runtime_error("Runtime.arm=both is not implemented yet (only 'right' is)");
    }
    throw std::runtime_error("Runtime.arm='" + arm + "' unknown (expected 'right')");
}

// arm right/left both use the single-Pika-unit topic; dual-arm ("both") needs a
// second pose source and is not implemented yet.
std::string resolve_pose_topic(const std::string& arm, const std::string& override_topic) {
    if (!override_topic.empty()) return override_topic;
    if (arm == "right" || arm == "left") return "/pika_pose";
    throw std::runtime_error("resolve_pose_topic: arm='both' is not supported yet");
}

CartesianControllerConfig load_cartesian_controller_config(const YAML::Node& root) {
    CartesianControllerConfig cfg;
    if (!root["CartesianController"]) return cfg;
    const YAML::Node n = root["CartesianController"];
    auto get_d = [&](const char* key, double def) {
        return n[key] ? n[key].as<double>() : def;
    };
    cfg.dls.kp_pos = get_d("kp_pos", cfg.dls.kp_pos);
    cfg.dls.kp_ori = get_d("kp_ori", cfg.dls.kp_ori);
    cfg.dls.lambda = get_d("lambda", cfg.dls.lambda);
    cfg.dls.max_linear_speed_mps = get_d("max_linear_speed_mps", cfg.dls.max_linear_speed_mps);
    cfg.dls.max_angular_speed_radps =
        get_d("max_angular_speed_radps", cfg.dls.max_angular_speed_radps);
    cfg.dls.max_joint_velocity_radps =
        get_d("max_joint_velocity_radps", cfg.dls.max_joint_velocity_radps);
    cfg.dls.soft_limit_margin_frac =
        get_d("soft_limit_margin_frac", cfg.dls.soft_limit_margin_frac);
    cfg.dls.max_tracking_error_rad = get_d("max_tracking_error_rad", cfg.dls.max_tracking_error_rad);
    if (n["use_nullspace"]) cfg.dls.use_nullspace = n["use_nullspace"].as<int>() != 0;
    cfg.dls.nullspace_gain = get_d("nullspace_gain", cfg.dls.nullspace_gain);
    if (n["debug_log"]) cfg.dls.debug_log = n["debug_log"].as<int>() != 0;
    cfg.max_gripper_speed = get_d("max_gripper_speed", cfg.max_gripper_speed);

    if (n["orientation_mode"] && n["orientation_mode"].IsScalar()) {
        const std::string mode = n["orientation_mode"].as<std::string>();
        if (mode == "hold" || mode == "position") {
            cfg.orientation_mode = OrientationMode::HOLD_CLUTCH_ORIENTATION;
            cfg.position_only_teleop = true;
        } else if (mode == "free") {
            cfg.orientation_mode = OrientationMode::FREE_ORIENTATION;
        } else {
            cfg.orientation_mode = OrientationMode::VR_ORIENTATION;
        }
    }
    return cfg;
}

struct PikaCartesianRates {
    double ik_update_hz = 120.0;
    double follower_hz = 1000.0;
};

PikaCartesianRates load_rates(const YAML::Node& root) {
    PikaCartesianRates rates;
    if (root["PikaCartesian"] && root["PikaCartesian"]["ik_update_hz"]) {
        rates.ik_update_hz = root["PikaCartesian"]["ik_update_hz"].as<double>();
    }
    if (root["PikaCartesian"] && root["PikaCartesian"]["follower_hz"]) {
        rates.follower_hz = root["PikaCartesian"]["follower_hz"].as<double>();
    }
    return rates;
}

// ---------------------------------------------------------------------------
// Small filesystem helpers
// ---------------------------------------------------------------------------

std::filesystem::path sensor_root_from_config(const std::filesystem::path& config_path) {
    std::error_code ec;
    std::filesystem::path config_dir = std::filesystem::absolute(config_path, ec).parent_path();
    if (config_dir.filename() == "config") {
        return config_dir.parent_path();
    }
    return config_dir;
}

std::string resolve_relative(const std::filesystem::path& base, const std::string& raw) {
    if (raw.empty() || raw[0] == '~') return raw;  // handled by consumers (e.g. '~' expansion)
    std::filesystem::path p(raw);
    if (p.is_absolute()) return raw;
    return (base / p).string();
}

bool file_contains(const std::string& path, const std::string& text) {
    std::ifstream file(path);
    if (!file) return false;
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str().find(text) != std::string::npos;
}

// Root link is fixed for this URDF; leaf link is the first matching TCP/hand/link
// candidate found for the given arm side.
std::string select_leaf_link(const std::string& urdf_path, const std::string& arm_side) {
    const std::string side = (arm_side == "left") ? "left" : "right";
    const std::vector<std::string> candidates = {
        "openarm_" + side + "_hand_tcp",
        "openarm_" + side + "_hand",
        "openarm_" + side + "_link8",
        "openarm_" + side + "_link7",
    };
    for (const auto& candidate : candidates) {
        if (file_contains(urdf_path, "link name=\"" + candidate + "\"")) {
            return candidate;
        }
    }
    return candidates.front();
}

// ---------------------------------------------------------------------------
// PikaPose conversion (PoseQueue only carries position + quaternion)
// ---------------------------------------------------------------------------

PikaPose pika_pose_from_sample(const PoseSample& s) {
    PikaPose p;
    p.time = s.t;
    p.x = s.pos[0];
    p.y = s.pos[1];
    p.z = s.pos[2];

    Eigen::Quaterniond q(s.rot[0], s.rot[1], s.rot[2], s.rot[3]);
    if (q.squaredNorm() < 1e-12) {
        q = Eigen::Quaterniond::Identity();
    }
    q.normalize();
    p.qw = q.w();
    p.qx = q.x();
    p.qy = q.y();
    p.qz = q.z();

    // ZYX intrinsic Euler (yaw, pitch, roll) to match CartesianMapper's
    // "vive" rotation_source convention (R = Rz(yaw) * Ry(pitch) * Rx(roll)).
    const Eigen::Vector3d ypr = q.toRotationMatrix().eulerAngles(2, 1, 0);
    p.yaw = ypr(0);
    p.pitch = ypr(1);
    p.roll = ypr(2);

    // No gripper/IMU-rate data on this topic; caller may still map a gripper
    // reference from grip_angle (stays at 0 => "closed" per config mapping).
    p.gx = p.gy = p.gz = 0.0;
    p.grip_angle = 0.0;
    p.grip_distance = 0.0;
    p.valid = true;
    return p;
}

// ---------------------------------------------------------------------------
// Cross-thread shared state
// ---------------------------------------------------------------------------

// Follower publishes its latest commanded joint positions here every tick so
// the (slower) admin/IK thread can seed IK from "where we are actually
// commanding" rather than the raw measured joints.
struct SharedQSeed {
    std::mutex mutex;
    std::vector<double> q;
    bool valid = false;
};

void safe_delete_openarm(openarm::can::socket::OpenArm* arm) {
    if (arm == nullptr) return;
    try {
        arm->disable_all();
    } catch (...) {
        // Destructors must not throw during shutdown.
    }
    delete arm;
}
struct OpenArmSafeDeleter {
    void operator()(openarm::can::socket::OpenArm* arm) const noexcept { safe_delete_openarm(arm); }
};

// ---------------------------------------------------------------------------
// FollowerArmThread @ follower_hz: interpolate segment cache -> CAN
// ---------------------------------------------------------------------------

class FollowerArmThread : public PeriodicTimerThread {
public:
    FollowerArmThread(std::shared_ptr<RobotSystemState> state, Control* control,
                      JointInterpolator* interpolator, JointSegmentCache* segment_cache,
                      SharedQSeed* q_seed, std::atomic<bool>* teleop_active, size_t arm_dof,
                      size_t hand_dof, double follower_hz)
        : PeriodicTimerThread(follower_hz),
          state_(std::move(state)),
          control_(control),
          interpolator_(interpolator),
          segment_cache_(segment_cache),
          q_seed_(q_seed),
          teleop_active_(teleop_active),
          arm_dof_(arm_dof),
          hand_dof_(hand_dof),
          dt_(1.0 / follower_hz) {}

    void set_steps_per_segment(int n) { steps_per_segment_ = std::max(1, n); }
    void set_grip_target(double grip) { grip_target_ = grip; }

protected:
    void on_timer() override {
        const bool active = teleop_active_ != nullptr && teleop_active_->load();

        if (!have_active_state_ || active != last_active_state_) {
            if (active) {
                const auto measured = state_->arm_state().get_all_responses();
                std::vector<double> measured_positions;
                measured_positions.reserve(std::min(arm_dof_, measured.size()));
                for (size_t i = 0; i < std::min(arm_dof_, measured.size()); ++i) {
                    measured_positions.push_back(measured[i].position);
                }
                if (measured_positions.size() == arm_dof_) {
                    interpolator_->reset(measured_positions);
                    q_cmd_prev_ = measured_positions;
                }
            }
            last_active_state_ = active;
            have_active_state_ = true;
        }

        if (segment_cache_ != nullptr) {
            std::vector<double> q_end;
            double grip_end = grip_target_;
            uint64_t seq = last_seq_;
            bool have_new_segment = false;
            {
                std::lock_guard<std::mutex> lock(segment_cache_->mutex);
                if (segment_cache_->valid && segment_cache_->seq != last_seq_) {
                    q_end = segment_cache_->q_end;
                    grip_end = segment_cache_->grip;
                    seq = segment_cache_->seq;
                    have_new_segment = true;
                }
            }
            if (active && have_new_segment) {
                interpolator_->set_segment(q_end, steps_per_segment_);
                last_seq_ = seq;
                grip_target_ = grip_end;
            }
        }

        std::vector<double> q_cmd;
        std::vector<double> dq_cmd;
        if (!active && !q_cmd_prev_.empty()) {
            interpolator_->hold_at(q_cmd_prev_);
        }
        interpolator_->step(dt_, &q_cmd, &dq_cmd);
        if (q_cmd.empty()) {
            control_->unilateral_step();
            return;
        }
        q_cmd_prev_ = q_cmd;

        if (q_seed_ != nullptr) {
            std::lock_guard<std::mutex> lock(q_seed_->mutex);
            q_seed_->q = q_cmd;
            q_seed_->valid = true;
        }

        std::vector<JointState> refs(arm_dof_ + hand_dof_);
        for (size_t i = 0; i < arm_dof_ && i < q_cmd.size(); ++i) {
            refs[i].position = q_cmd[i];
            refs[i].velocity = i < dq_cmd.size() ? dq_cmd[i] : 0.0;
        }
        if (hand_dof_ > 0) {
            refs[arm_dof_].position = grip_target_;
        }
        state_->set_all_references(refs);

        control_->unilateral_step();
    }

private:
    std::shared_ptr<RobotSystemState> state_;
    Control* control_;
    JointInterpolator* interpolator_;
    JointSegmentCache* segment_cache_;
    SharedQSeed* q_seed_;
    std::atomic<bool>* teleop_active_;
    size_t arm_dof_;
    size_t hand_dof_;
    double dt_;
    int steps_per_segment_ = 1;
    uint64_t last_seq_ = 0;
    double grip_target_ = 0.0;
    std::vector<double> q_cmd_prev_;
    bool have_active_state_ = false;
    bool last_active_state_ = false;
};

// ---------------------------------------------------------------------------
// PikaAdminThread @ ik_update_hz: pose queue -> mapper -> IK -> segment cache
// ---------------------------------------------------------------------------

class PikaAdminThread : public PeriodicTimerThread {
public:
    PikaAdminThread(std::shared_ptr<RobotSystemState> state, PoseQueue* pose_queue,
                    CartesianMapper* mapper, Dynamics* dynamics, IkSolver* ik_solver,
                    JointSegmentCache* segment_cache, SharedQSeed* q_seed,
                    std::atomic<bool>* teleop_active, std::atomic<bool>* teleop_rezero,
                    std::atomic<bool>* teleop_stop_req, TcpRelativeCsvWriter* csv_writer,
                    size_t arm_dof, double ik_update_hz)
        : PeriodicTimerThread(ik_update_hz),
          state_(std::move(state)),
          pose_queue_(pose_queue),
          mapper_(mapper),
          dynamics_(dynamics),
          ik_solver_(ik_solver),
          segment_cache_(segment_cache),
          q_seed_(q_seed),
          teleop_active_(teleop_active),
          teleop_rezero_(teleop_rezero),
          teleop_stop_req_(teleop_stop_req),
          csv_writer_(csv_writer),
          arm_dof_(arm_dof),
          dt_ik_(1.0 / ik_update_hz) {}

protected:
    // This thread only reads ROS queues / does math; no CAN I/O, no need for RT
    // scheduling priority (leave that budget to the 1kHz follower thread).
    void before_start() override {}

    void on_timer() override {
        const auto admin_now = std::chrono::steady_clock::now();

        if (teleop_stop_req_ != nullptr && teleop_stop_req_->exchange(false)) {
            if (teleop_active_ != nullptr) teleop_active_->store(false);
            if (mapper_ != nullptr) mapper_->reset_home();
            if (teleop_rezero_ != nullptr) teleop_rezero_->store(false);
            std::cout << "[pika] Teleop STOPPED — holding current pose (press p to re-zero)."
                      << std::endl;
        }

        bool rezero_edge = false;
        if (teleop_rezero_ != nullptr && teleop_rezero_->exchange(false)) {
            if (mapper_ != nullptr) mapper_->reset_home();
            rezero_edge = true;
            if (teleop_active_ != nullptr) teleop_active_->store(true);
            std::cout << "[pika] Teleop STARTED — relative home = pose at this press."
                      << std::endl;
        }

        const bool active = teleop_active_ != nullptr && teleop_active_->load();
        if (!active) {
            if (segment_cache_ != nullptr) {
                std::lock_guard<std::mutex> lock(segment_cache_->mutex);
                segment_cache_->valid = false;
            }
            return;
        }

        std::optional<PoseSample> latest;
        if (pose_queue_ != nullptr) {
            std::optional<PoseSample> sample;
            while ((sample = pose_queue_->try_pop()).has_value()) {
                latest = sample;
            }
        }
        if (latest.has_value()) {
            last_pose_sample_ = latest;
            last_pose_received_time_ = admin_now;
        } else if (last_pose_sample_.has_value() &&
                   last_pose_received_time_.time_since_epoch().count() != 0 &&
                   admin_now - last_pose_received_time_ <= std::chrono::milliseconds(100)) {
            // Keep the 120 Hz IK/segment stream continuous across the locator's
            // short, irregular publish gaps.  The desired pose is unchanged,
            // so DLS continues closing smoothly instead of the interpolator
            // reaching its endpoint and dropping velocity to zero.
            latest = last_pose_sample_;
        } else {
            // A genuinely stale pose must not be followed indefinitely.
            return;
        }

        const PikaPose pika_pose = pika_pose_from_sample(latest.value());
        print_pika_pose(pika_pose);

        // IK seed: on the rising edge of "active" (rezero), seed from measured
        // joints; otherwise use the Follower thread's latest commanded snapshot.
        std::vector<double> q_seed;
        if (rezero_edge) {
            q_seed = measured_arm_positions();
            if (ik_solver_ != nullptr && !q_seed.empty()) ik_solver_->reset(q_seed);
        } else if (q_seed_ != nullptr) {
            std::lock_guard<std::mutex> lock(q_seed_->mutex);
            if (q_seed_->valid) q_seed = q_seed_->q;
        }
        if (q_seed.empty()) {
            q_seed = measured_arm_positions();
        }

        if (!mapper_->home_set()) {
            const std::vector<double> q_measured = measured_arm_positions();
            Eigen::Matrix3d r_cur = Eigen::Matrix3d::Identity();
            Eigen::Vector3d p_cur = Eigen::Vector3d::Zero();
            if (dynamics_ != nullptr && q_measured.size() >= arm_dof_) {
                dynamics_->GetEECordinate(q_measured.data(), r_cur, p_cur);
            }
            mapper_->set_home(pika_pose, r_cur, p_cur);
        }

        Eigen::Matrix3d r_des = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p_des = Eigen::Vector3d::Zero();
        double grip_ref = 0.0;
        mapper_->map(pika_pose, q_seed, r_des, p_des, grip_ref);

        if (csv_writer_ != nullptr && csv_writer_->enabled()) {
            const double now_wall = std::chrono::duration<double>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count();
            csv_writer_->record(p_des, r_des, now_wall);
        }

        if (ik_solver_ == nullptr || q_seed.size() != arm_dof_) {
            return;
        }

        const Eigen::Quaterniond q_des(r_des);
        std::vector<double> q_end;
        double grip_out = grip_ref;
        const bool ok =
            ik_solver_->step(q_seed, p_des, q_des, grip_ref, dt_ik_, &q_end, &grip_out);
        if (!ok) {
            std::cerr << "[pika] WARN: IK step failed; holding last commanded joints"
                      << std::endl;
        }

        if (segment_cache_ != nullptr) {
            std::lock_guard<std::mutex> lock(segment_cache_->mutex);
            segment_cache_->q_end = q_end;
            segment_cache_->grip = grip_out;
            segment_cache_->seq += 1;
            segment_cache_->valid = true;
        }
    }

private:
    std::vector<double> measured_arm_positions() const {
        const auto resp = state_->arm_state().get_all_responses();
        std::vector<double> q(resp.size());
        for (size_t i = 0; i < resp.size(); ++i) q[i] = resp[i].position;
        return q;
    }

    void print_pika_pose(const PikaPose& pose) {
        const auto now = std::chrono::steady_clock::now();
        if (now < next_print_time_) return;
        next_print_time_ = now + std::chrono::milliseconds(200);
        std::cout << std::fixed << std::setprecision(4) << "[pika] pos=(" << pose.x << ","
                  << pose.y << "," << pose.z << ")" << std::endl;
    }

    std::shared_ptr<RobotSystemState> state_;
    PoseQueue* pose_queue_;
    CartesianMapper* mapper_;
    Dynamics* dynamics_;
    IkSolver* ik_solver_;
    JointSegmentCache* segment_cache_;
    SharedQSeed* q_seed_;
    std::atomic<bool>* teleop_active_;
    std::atomic<bool>* teleop_rezero_;
    std::atomic<bool>* teleop_stop_req_;
    TcpRelativeCsvWriter* csv_writer_;
    size_t arm_dof_;
    double dt_ik_;
    std::chrono::steady_clock::time_point next_print_time_;
    std::optional<PoseSample> last_pose_sample_;
    std::chrono::steady_clock::time_point last_pose_received_time_{};
};

}  // namespace

struct TeleopSession::Impl {
    std::unique_ptr<openarm::can::socket::OpenArm, OpenArmSafeDeleter> follower_openarm;
    std::shared_ptr<RobotSystemState> follower_state;
    std::unique_ptr<Dynamics> dynamics;
    std::unique_ptr<Control> control;
    std::unique_ptr<IkSolver> ik_solver;
    std::unique_ptr<JointInterpolator> interpolator;
    std::unique_ptr<CartesianMapper> mapper;
    std::unique_ptr<TcpRelativeCsvWriter> csv_writer;
    std::unique_ptr<PoseQueue> pose_queue;
    std::unique_ptr<PikaPoseRosIO> pose_io;
    std::unique_ptr<FollowerArmThread> follower_thread;
    std::unique_ptr<PikaAdminThread> admin_thread;
    std::unique_ptr<JointSegmentCache> segment_cache;
    std::unique_ptr<SharedQSeed> q_seed;

    RuntimeConfig runtime;
    PikaCartesianRates rates;
    size_t arm_motor_num = 0;
    size_t hand_motor_num = 0;
    int steps_per_segment = 1;
    bool control_started = false;
};

TeleopSession::TeleopSession() : impl_(std::make_unique<Impl>()), pose_health_(std::make_unique<PoseHealthMonitor>()) {}

TeleopSession::~TeleopSession() { stop(); }

bool TeleopSession::init(int argc, char** argv, const std::string& config_path,
                         const std::string& vive_csv_path) {
    if (!std::filesystem::exists(config_path)) {
        std::cerr << "[ERROR] config file not found: " << config_path << std::endl;
        return false;
    }
    const std::filesystem::path sensor_root = sensor_root_from_config(config_path);

    try {
        YAML::Node root = YAML::LoadFile(config_path);
        impl_->runtime = load_runtime_config(root);
        validate_arm_side(impl_->runtime.arm);
        const std::string pose_topic =
            resolve_pose_topic(impl_->runtime.arm, impl_->runtime.pose_topic);

        const std::string urdf_path = resolve_relative(sensor_root, impl_->runtime.urdf);
        if (!std::filesystem::exists(urdf_path)) {
            std::cerr << "[ERROR] URDF not found: " << urdf_path << std::endl;
            return false;
        }
        const std::string root_link = "openarm_body_link0";
        const std::string leaf_link = select_leaf_link(urdf_path, impl_->runtime.arm);

        printOpenArmBanner();
        std::cout << "=== OpenArm Pika Cartesian (all_telop) ===" << std::endl;
        std::cout << "Config         : " << config_path << std::endl;
        std::cout << "Arm side       : " << impl_->runtime.arm << std::endl;
        std::cout << "Follower CAN   : " << impl_->runtime.can_interface << std::endl;
        std::cout << "Pose topic     : " << pose_topic << std::endl;
        std::cout << "URDF path      : " << urdf_path << std::endl;
        std::cout << "Vive CSV       : " << vive_csv_path << std::endl;

        setenv("OPENARM_SKIP_GRIPPER", impl_->runtime.skip_gripper ? "1" : "0", 1);

        impl_->dynamics = std::make_unique<Dynamics>(urdf_path, root_link, leaf_link);
        if (!impl_->dynamics->Init()) {
            std::cerr << "[ERROR] Dynamics Init failed" << std::endl;
            return false;
        }
        const size_t arm_dof = impl_->dynamics->GetDof();
        (void)arm_dof;

        if (!rclcpp::ok()) {
            rclcpp::init(argc, argv);
            ros_initialized_ = true;
        }

        std::cout << "=== Initializing Follower OpenArm ===" << std::endl;
        impl_->follower_openarm.reset(
            openarm_init::OpenArmInitializer::initialize_openarm(impl_->runtime.can_interface,
                                                                 true));

        impl_->arm_motor_num = impl_->follower_openarm->get_arm().get_motors().size();
        impl_->hand_motor_num = impl_->follower_openarm->get_gripper().get_motors().size();
        std::cout << "arm motor num  : " << impl_->arm_motor_num << std::endl;
        std::cout << "hand motor num : " << impl_->hand_motor_num << std::endl;

        impl_->follower_state =
            std::make_shared<RobotSystemState>(impl_->arm_motor_num, impl_->hand_motor_num);

        impl_->rates = load_rates(root);
        impl_->control = std::make_unique<Control>(
            impl_->follower_openarm.get(), impl_->dynamics.get(), impl_->dynamics.get(),
            impl_->follower_state, 1.0 / impl_->rates.follower_hz, ROLE_FOLLOWER,
            impl_->runtime.arm, impl_->arm_motor_num, impl_->hand_motor_num);

        YamlLoader follower_loader(config_path);
        impl_->control->SetParameter(follower_loader.get_vector("FollowerArmParam", "Kp"),
                                     follower_loader.get_vector("FollowerArmParam", "Kd"),
                                     follower_loader.get_vector("FollowerArmParam", "Fc"),
                                     follower_loader.get_vector("FollowerArmParam", "k"),
                                     follower_loader.get_vector("FollowerArmParam", "Fv"),
                                     follower_loader.get_vector("FollowerArmParam", "Fo"));
        impl_->control->SetSafetyFilter(std::make_unique<SafetyFilter>(
            make_default_safety_config(impl_->arm_motor_num + impl_->hand_motor_num)));

        impl_->control->AdjustPosition();
        impl_->control->ResetSafetyFilter(impl_->follower_state->get_all_references());

        std::vector<double> q0(impl_->arm_motor_num);
        {
            const auto arm_resp = impl_->follower_state->arm_state().get_all_responses();
            for (size_t i = 0; i < impl_->arm_motor_num && i < arm_resp.size(); ++i) {
                q0[i] = arm_resp[i].position;
            }
        }

        CartesianControllerConfig cc_cfg = load_cartesian_controller_config(root);
        impl_->ik_solver = create_ik_solver(impl_->runtime.ik, impl_->dynamics.get(), cc_cfg);
        if (!impl_->ik_solver) {
            std::cerr << "[ERROR] unknown ik solver '" << impl_->runtime.ik << "'" << std::endl;
            return false;
        }
        impl_->ik_solver->reset(q0);

        impl_->steps_per_segment = std::max(
            1, static_cast<int>(std::lround(impl_->rates.follower_hz / impl_->rates.ik_update_hz)));
        impl_->interpolator =
            create_joint_interpolator(impl_->runtime.interpolator, impl_->steps_per_segment);
        if (!impl_->interpolator) {
            std::cerr << "[ERROR] unknown interpolator '" << impl_->runtime.interpolator << "'"
                      << std::endl;
            return false;
        }
        impl_->interpolator->reset(q0);

        impl_->mapper = std::make_unique<CartesianMapper>();
        impl_->mapper->load_config(config_path);

        impl_->csv_writer = std::make_unique<TcpRelativeCsvWriter>();
        impl_->csv_writer->set_enabled(impl_->runtime.record_tcp_csv);
        if (impl_->runtime.record_tcp_csv) {
            const std::string csv_path =
                vive_csv_path.empty() ? impl_->runtime.record_tcp_csv_path : vive_csv_path;
            if (!impl_->csv_writer->open(csv_path)) {
                std::cerr << "[WARN] failed to open TCP CSV; disabling recording" << std::endl;
                impl_->csv_writer->set_enabled(false);
            }
        }

        impl_->pose_queue = std::make_unique<PoseQueue>(2048);
        impl_->pose_io = std::make_unique<PikaPoseRosIO>();
        if (!impl_->pose_io->start(pose_topic, impl_->pose_queue.get(), pose_health_.get())) {
            std::cerr << "[ERROR] failed to start pose subscriber on " << pose_topic << std::endl;
            return false;
        }

        ready_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] TeleopSession::init: " << e.what() << std::endl;
        stop();
        return false;
    }
}

bool TeleopSession::runStaticCheck() {
    if (!ready_ || !pose_health_) return false;
    (void)pose_health_->run_static_check(keep_running, PoseHealthMonitor::StaticOptions{});
    if (impl_->pose_queue) {
        impl_->pose_queue->clear();
    }
    return keep_running.load();
}

void TeleopSession::startControl(std::atomic<bool>* session_active,
                                 std::atomic<bool>* session_start,
                                 std::atomic<bool>* session_stop) {
    if (!ready_ || impl_->control_started) return;

    impl_->segment_cache = std::make_unique<JointSegmentCache>();
    impl_->q_seed = std::make_unique<SharedQSeed>();

    impl_->follower_thread = std::make_unique<FollowerArmThread>(
        impl_->follower_state, impl_->control.get(), impl_->interpolator.get(),
        impl_->segment_cache.get(), impl_->q_seed.get(), session_active, impl_->arm_motor_num,
        impl_->hand_motor_num, impl_->rates.follower_hz);
    impl_->follower_thread->set_steps_per_segment(impl_->steps_per_segment);
    if (impl_->hand_motor_num > 0) {
        const auto hand_resp = impl_->follower_state->hand_state().get_all_responses();
        impl_->follower_thread->set_grip_target(hand_resp.empty() ? 0.0
                                                                  : hand_resp.front().position);
    }

    impl_->admin_thread = std::make_unique<PikaAdminThread>(
        impl_->follower_state, impl_->pose_queue.get(), impl_->mapper.get(),
        impl_->dynamics.get(), impl_->ik_solver.get(), impl_->segment_cache.get(),
        impl_->q_seed.get(), session_active, session_start, session_stop, impl_->csv_writer.get(),
        impl_->arm_motor_num, impl_->rates.ik_update_hz);

    std::cout << "[pika] Admin/IK      : " << impl_->rates.ik_update_hz << " Hz" << std::endl;
    std::cout << "[pika] Follower/CAN  : " << impl_->rates.follower_hz << " Hz ("
              << impl_->steps_per_segment << " interp steps/segment)" << std::endl;

    impl_->follower_thread->start_thread();
    impl_->admin_thread->start_thread();
    impl_->control_started = true;
}

void TeleopSession::stop() {
    if (impl_) {
        if (impl_->follower_thread) impl_->follower_thread->stop_thread();
        if (impl_->admin_thread) impl_->admin_thread->stop_thread();
        if (impl_->pose_io) impl_->pose_io->stop();
        impl_->follower_thread.reset();
        impl_->admin_thread.reset();
        impl_->pose_io.reset();
        impl_->csv_writer.reset();
        impl_->follower_openarm.reset();
        impl_->control.reset();
        impl_->control_started = false;
    }
    if (ros_initialized_ && rclcpp::ok()) {
        rclcpp::shutdown();
        ros_initialized_ = false;
    }
    ready_ = false;
}

