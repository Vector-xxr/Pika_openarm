#include <io/pika_pose_ros.hpp>

#include <pose_health_monitor.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <iostream>
#include <string>

namespace {

double now_wall_sec() {
    using clock = std::chrono::system_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

PoseSample from_msg(const geometry_msgs::msg::PoseStamped& msg) {
    PoseSample s;
    const double stamp = static_cast<double>(msg.header.stamp.sec) +
                          1e-9 * static_cast<double>(msg.header.stamp.nanosec);
    s.t = (stamp > 1.0) ? stamp : now_wall_sec();
    s.pos = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
    // PoseStamped orientation is xyzw -> PoseSample stores wxyz.
    s.rot = {msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z};
    return s;
}

std::atomic<uint64_t> g_node_counter{0};

}  // namespace

class PikaPoseRosIO::Impl : public rclcpp::Node {
public:
    Impl(const std::string& topic, PoseQueue* queue, PoseHealthMonitor* health)
        : Node("pika_pose_ros_io_" + std::to_string(g_node_counter.fetch_add(1))),
          queue_(queue),
          health_(health) {
        sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_pose(*msg); });
    }

private:
    void on_pose(const geometry_msgs::msg::PoseStamped& msg) {
        const PoseSample sample = from_msg(msg);

        bool accepted_for_control = true;
        if (health_ != nullptr) {
            accepted_for_control = health_->on_sample(sample);
        }
        if (queue_ != nullptr && accepted_for_control) {
            queue_->push(sample);
        }
    }

    PoseQueue* queue_;
    PoseHealthMonitor* health_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};

PikaPoseRosIO::~PikaPoseRosIO() { stop(); }

bool PikaPoseRosIO::start(const std::string& topic, PoseQueue* queue,
                          PoseHealthMonitor* health) {
    if (running_.load()) {
        stop();
    }
    if (queue == nullptr) {
        std::cerr << "[pika_pose_ros] ERROR: null PoseQueue" << std::endl;
        return false;
    }
    try {
        impl_ = std::make_shared<Impl>(topic, queue, health);
    } catch (const std::exception& e) {
        std::cerr << "[pika_pose_ros] ERROR: failed to create node: " << e.what() << std::endl;
        return false;
    }

    running_.store(true);
    std::shared_ptr<Impl> node = impl_;
    std::atomic<bool>* running_flag = &running_;
    spin_thread_ = std::thread([node, running_flag]() {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        while (running_flag->load() && rclcpp::ok()) {
            executor.spin_some(std::chrono::milliseconds(50));
        }
        executor.remove_node(node);
    });

    std::cout << "[pika_pose_ros] subscribed to " << topic << std::endl;
    return true;
}

void PikaPoseRosIO::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    impl_.reset();
}
