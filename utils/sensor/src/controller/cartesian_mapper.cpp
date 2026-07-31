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

#include <controller/cartesian_mapper.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

#include <yaml-cpp/yaml.h>

// Convert Euler angles (roll, pitch, yaw) to rotation matrix (ZYX convention)
static Eigen::Matrix3d euler_to_rotation(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    return (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
}

// Convert quaternion (w, x, y, z) to rotation matrix
static Eigen::Matrix3d quat_to_rotation(double qw, double qx, double qy, double qz) {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize();
    return q.toRotationMatrix();
}

void CartesianMapper::set_home(const PikaPose &pika_home, const Eigen::Matrix3d &ee_home_R,
                                const Eigen::Vector3d &ee_home_p) {
    pika_pos_home_ = Eigen::Vector3d(pika_home.x, pika_home.y, pika_home.z);

    // Record pika rotation at home based on configured source
    if (config_.rotation_source == "imu") {
        pika_rot_home_ = quat_to_rotation(pika_home.qw, pika_home.qx, pika_home.qy, pika_home.qz);
    } else {
        pika_rot_home_ = euler_to_rotation(pika_home.roll, pika_home.pitch, pika_home.yaw);
    }

    ee_pos_home_ = ee_home_p;
    ee_rot_home_ = ee_home_R;
    accepted_relative_position_.setZero();
    accepted_relative_position_valid_ = true;
    accepted_relative_rot_ = Eigen::Matrix3d::Identity();
    accepted_relative_rot_valid_ = true;
    last_desired_position_ = ee_home_p;
    last_desired_rotation_ = ee_home_R;
    last_desired_pose_valid_ = true;
    home_set_ = true;
    std::cout << "[cartesian-mapper] home set: pika_pos=(" << pika_pos_home_.x() << ","
              << pika_pos_home_.y() << "," << pika_pos_home_.z() << ") ee_pos=(" << ee_pos_home_.x()
              << "," << ee_pos_home_.y() << "," << ee_pos_home_.z() << ")" << std::endl;
}

void CartesianMapper::map(const PikaPose &pika_pose, const std::vector<double> &current_q,
                          Eigen::Matrix3d &R_des_out, Eigen::Vector3d &p_des_out,
                          double &grip_ref_out) {
    // --- Position mapping ---
    // p_des = R_base * (p_vive - p_home) * scale + p_ee_home
    Eigen::Vector3d pika_pos(pika_pose.x, pika_pose.y, pika_pose.z);
    Eigen::Vector3d delta = pika_pos - pika_pos_home_;

    // Stateful positional deadband ("play" operator): reject small reversals
    // around the last accepted displacement at any point in the workspace,
    // while retaining at most one deadzone radius of spatial lag.
    if (!accepted_relative_position_valid_) {
        accepted_relative_position_ = delta;
        accepted_relative_position_valid_ = true;
    }
    const Eigen::Vector3d position_from_accepted = delta - accepted_relative_position_;
    const double position_from_accepted_norm = position_from_accepted.norm();
    if (config_.position_deadzone <= 0.0) {
        accepted_relative_position_ = delta;
    } else if (std::isfinite(position_from_accepted_norm) &&
               position_from_accepted_norm > config_.position_deadzone) {
        accepted_relative_position_ +=
            position_from_accepted *
            ((position_from_accepted_norm - config_.position_deadzone) /
             position_from_accepted_norm);
    }
    delta = accepted_relative_position_;

    delta = config_.base_offset * delta * config_.position_scale;
    p_des_out = ee_pos_home_ + delta;

    // Clamp to workspace limits, but always include the physical TCP pose that
    // was captured as home.  This preserves the relative-control invariant:
    // zero Pika displacement must command exactly ee_pos_home_.  If the arm was
    // already outside a configured bound at activation, it may move back toward
    // the configured workspace but not farther beyond its captured home.
    for (int i = 0; i < 3; ++i) {
        const double effective_min = std::min(config_.position_min(i), ee_pos_home_(i));
        const double effective_max = std::max(config_.position_max(i), ee_pos_home_(i));
        p_des_out(i) = std::clamp(p_des_out(i), effective_min, effective_max);
    }

    if (last_desired_pose_valid_ && config_.max_position_step_per_frame > 0.0) {
        Eigen::Vector3d output_step = p_des_out - last_desired_position_;
        const double output_step_norm = output_step.norm();
        if (output_step_norm > config_.max_position_step_per_frame) {
            output_step *= config_.max_position_step_per_frame / output_step_norm;
            p_des_out = last_desired_position_ + output_step;
        }
    }
    // --- Rotation mapping ---
    // Preserve local-axis correspondence:
    // R_des = R_ee_home * (R_pika_home^-1 * R_pika)
    Eigen::Matrix3d R_pika;
    if (config_.rotation_source == "imu") {
        R_pika = quat_to_rotation(pika_pose.qw, pika_pose.qx, pika_pose.qy, pika_pose.qz);
    } else {
        R_pika = euler_to_rotation(pika_pose.roll, pika_pose.pitch, pika_pose.yaw);
    }

    // Relative rotation expressed in the Pika home/local frame.
    // Apply it on the right so Pika-local +X/+Y/+Z rotations map to
    // TCP-local +X/+Y/+Z rotations. Optional rotation_scale shrinks the
    // relative angle about the same axis (1.0 = 1:1).
    Eigen::Matrix3d R_relative = pika_rot_home_.transpose() * R_pika;

    // Stateful angular deadband ("play" operator): keep the accepted target
    // while raw tracker jitter stays inside the deadzone.  For intentional
    // motion beyond it, advance continuously while retaining only a bounded
    // deadzone-sized lag, avoiding a step when motion resumes.
    if (!accepted_relative_rot_valid_) {
        accepted_relative_rot_ = R_relative;
        accepted_relative_rot_valid_ = true;
    }
    const Eigen::Matrix3d R_from_accepted = accepted_relative_rot_.transpose() * R_relative;
    const Eigen::AngleAxisd delta_from_accepted(R_from_accepted);
    const double delta_angle = delta_from_accepted.angle();
    if (config_.rotation_deadzone <= 0.0) {
        accepted_relative_rot_ = R_relative;
    } else if (std::isfinite(delta_angle) && delta_angle > config_.rotation_deadzone &&
               delta_from_accepted.axis().allFinite()) {
        const double accepted_step = delta_angle - config_.rotation_deadzone;
        accepted_relative_rot_ =
            accepted_relative_rot_ *
            Eigen::AngleAxisd(accepted_step, delta_from_accepted.axis()).toRotationMatrix();
    }
    R_relative = accepted_relative_rot_;

    if (std::abs(config_.rotation_scale - 1.0) > 1e-9) {
        Eigen::AngleAxisd aa(R_relative);
        double angle = aa.angle();
        if (std::isfinite(angle) && angle > 1e-9 && aa.axis().allFinite()) {
            R_relative =
                Eigen::AngleAxisd(angle * config_.rotation_scale, aa.axis()).toRotationMatrix();
        }
    }
    R_des_out = ee_rot_home_ * R_relative;
    if (last_desired_pose_valid_ && config_.max_rotation_step_per_frame > 0.0) {
        const Eigen::Matrix3d output_delta = last_desired_rotation_.transpose() * R_des_out;
        const Eigen::AngleAxisd output_aa(output_delta);
        if (std::isfinite(output_aa.angle()) &&
            output_aa.angle() > config_.max_rotation_step_per_frame &&
            output_aa.axis().allFinite()) {
            R_des_out =
                last_desired_rotation_ *
                Eigen::AngleAxisd(config_.max_rotation_step_per_frame, output_aa.axis())
                    .toRotationMatrix();
        }
    }

    last_desired_position_ = p_des_out;
    last_desired_rotation_ = R_des_out;
    last_desired_pose_valid_ = true;

    // --- Gripper mapping ---
    // Linear map: pika [0, 1.67] -> openarm [min, max]
    double grip_angle = pika_pose.grip_angle;
    if (last_grip_angle_valid_) {
        if (std::abs(grip_angle - last_grip_angle_) < config_.gripper_deadzone) {
            grip_angle = last_grip_angle_;
        }
        const double max_step = config_.gripper_max_step_per_frame;
        if (max_step > 0.0) {
            const double d = grip_angle - last_grip_angle_;
            if (std::abs(d) > max_step) {
                grip_angle = last_grip_angle_ + std::copysign(max_step, d);
            }
        }
    }
    last_grip_angle_ = grip_angle;
    last_grip_angle_valid_ = true;

    double normalized = std::clamp(grip_angle / config_.pika_gripper_max, 0.0, 1.0);
    grip_ref_out =
        config_.openarm_gripper_min + normalized * (config_.openarm_gripper_max - config_.openarm_gripper_min);
}

void CartesianMapper::load_config(const std::string &yaml_path) {
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root["PikaCartesian"]) {
            std::cerr << "[cartesian-mapper] WARNING: 'PikaCartesian' node not found in "
                      << yaml_path << ", using defaults" << std::endl;
            return;
        }
        YAML::Node n = root["PikaCartesian"];

        if (n["position_scale"]) config_.position_scale = n["position_scale"].as<double>();
        if (n["rotation_scale"]) config_.rotation_scale = n["rotation_scale"].as<double>();
        if (n["position_deadzone"]) config_.position_deadzone = n["position_deadzone"].as<double>();
        if (n["max_position_step_per_frame"]) {
            config_.max_position_step_per_frame = n["max_position_step_per_frame"].as<double>();
        }
        if (n["position_min"]) {
            auto v = n["position_min"].as<std::vector<double>>();
            if (v.size() >= 3) config_.position_min = Eigen::Vector3d(v[0], v[1], v[2]);
        }
        if (n["position_max"]) {
            auto v = n["position_max"].as<std::vector<double>>();
            if (v.size() >= 3) config_.position_max = Eigen::Vector3d(v[0], v[1], v[2]);
        }
        if (n["rotation_source"]) config_.rotation_source = n["rotation_source"].as<std::string>();
        if (n["rotation_deadzone"]) {
            config_.rotation_deadzone = n["rotation_deadzone"].as<double>();
        }
        if (n["max_rotation_step_per_frame"]) {
            config_.max_rotation_step_per_frame =
                n["max_rotation_step_per_frame"].as<double>();
        }
        if (n["base_offset_rpy"]) {
            auto v = n["base_offset_rpy"].as<std::vector<double>>();
            if (v.size() >= 3) {
                config_.base_offset =
                    (Eigen::AngleAxisd(v[2], Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(v[1], Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(v[0], Eigen::Vector3d::UnitX()))
                        .toRotationMatrix();
            }
        }
        if (n["tool_offset_rpy"]) {
            auto v = n["tool_offset_rpy"].as<std::vector<double>>();
            if (v.size() >= 3) {
                config_.tool_offset =
                    (Eigen::AngleAxisd(v[2], Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(v[1], Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(v[0], Eigen::Vector3d::UnitX()))
                        .toRotationMatrix();
            }
        }
        if (n["pika_gripper_max"]) config_.pika_gripper_max = n["pika_gripper_max"].as<double>();
        if (n["openarm_gripper_max"]) config_.openarm_gripper_max = n["openarm_gripper_max"].as<double>();
        if (n["openarm_gripper_min"]) config_.openarm_gripper_min = n["openarm_gripper_min"].as<double>();
        if (n["gripper_deadzone"]) config_.gripper_deadzone = n["gripper_deadzone"].as<double>();
        if (n["gripper_max_step_per_frame"]) {
            config_.gripper_max_step_per_frame = n["gripper_max_step_per_frame"].as<double>();
        }
        if (n["ik_max_iter"]) config_.ik_max_iter = n["ik_max_iter"].as<int>();
        if (n["ik_eps"]) config_.ik_eps = n["ik_eps"].as<double>();
        if (n["ik_lambda"]) config_.ik_lambda = n["ik_lambda"].as<double>();
        if (n["ik_alpha"]) config_.ik_alpha = n["ik_alpha"].as<double>();
        if (n["ik_use_nullspace"]) config_.ik_use_nullspace = n["ik_use_nullspace"].as<int>() != 0;
        if (n["ik_nullspace_gain"]) config_.ik_nullspace_gain = n["ik_nullspace_gain"].as<double>();
        if (n["ik_max_joint_step"]) config_.ik_max_joint_step = n["ik_max_joint_step"].as<double>();
        if (n["ik_joint_weights"]) {
            config_.ik_joint_weights = n["ik_joint_weights"].as<std::vector<double>>();
        }
        if (n["position_weight"]) config_.position_weight = n["position_weight"].as<double>();
        if (n["rotation_weight"]) config_.rotation_weight = n["rotation_weight"].as<double>();

        std::cout << "[cartesian-mapper] config loaded from " << yaml_path << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "[cartesian-mapper] WARNING: failed to load config " << yaml_path << ": "
                  << e.what() << ", using defaults" << std::endl;
    }
}
