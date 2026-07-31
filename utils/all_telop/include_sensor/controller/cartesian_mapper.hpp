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

#pragma once

#include <cstdint>
#include <Eigen/Dense>
#include "io/pika_pose.hpp"
#include <string>
#include <vector>

// Configuration for mapping Pika Vive/IMU pose to OpenArm EE desired pose.
struct CartesianMapperConfig {
    // --- Position mapping ---
    double position_scale = 1.0;           // scale factor Vive->arm
    double position_deadzone = 0.005;      // deadzone [m] to suppress jitter
    double max_position_step_per_frame = 0.005;  // max desired TCP step per map() [m]
    Eigen::Vector3d position_min = Eigen::Vector3d(0.1, -0.4, 0.0);   // workspace limits
    Eigen::Vector3d position_max = Eigen::Vector3d(0.8, 0.4, 0.8);

    // --- Rotation mapping ---
    // Scale relative Vive/IMU rotation angle about its axis (1.0 = 1:1).
    double rotation_scale = 1.0;
    // Stateful angular deadband [rad] used to suppress tracker orientation jitter.
    double rotation_deadzone = 0.01;
    // Max desired TCP angular step per map() [rad].
    double max_rotation_step_per_frame = 0.0125;
    // R_base_offset: rotation from Vive frame to OpenArm base frame
    // Default = identity (must be calibrated for your setup)
    Eigen::Matrix3d base_offset = Eigen::Matrix3d::Identity();
    // R_tool_offset: rotation from IMU/Vive frame to EE tool frame
    Eigen::Matrix3d tool_offset = Eigen::Matrix3d::Identity();

    // Choose rotation source: "vive" (from RPY) or "imu" (from quaternion)
    std::string rotation_source = "vive";

    // --- Gripper mapping ---
    double pika_gripper_max = 1.67;        // pika gripper angle range [rad]
    double openarm_gripper_max = 0.5;      // openarm gripper max position [rad]
    double openarm_gripper_min = 0.0;      // openarm gripper min position [rad]
    double gripper_deadzone = 0.03;        // ignore small angle jitter [rad]
    double gripper_max_step_per_frame = 0.15;  // max |Δangle| per map() call [rad]

    // --- IK parameters ---
    int ik_max_iter = 50;
    double ik_eps = 1e-4;
    double ik_lambda = 0.05;
    double ik_alpha = 1.0;
    bool ik_use_nullspace = false;
    double ik_nullspace_gain = 0.01;
    // Max |Δq| per IK output frame; scale whole vector if exceeded [rad]
    double ik_max_joint_step = 0.35;
    // Joint weights for weighted DLS (larger = move less). Empty => identity.
    std::vector<double> ik_joint_weights;
    // Task-space DLS weights for position and rotation errors.
    double position_weight = 0.5;
    double rotation_weight = 0.5;

    // --- Velocity reference ---
    bool compute_velocity = true;          // estimate dq_ref from consecutive q_ref
};

// Maps Pika Vive/IMU pose to OpenArm end-effector desired pose and gripper reference.
// Handles:
//   1. Position: relative mapping with scale, deadzone, limits
//   2. Rotation: Vive RPY or IMU quaternion with base/tool offsets
//   3. Gripper: linear mapping from pika angle to openarm range
class CartesianMapper {
public:
    CartesianMapper() = default;

    void set_config(const CartesianMapperConfig &config) { config_ = config; }
    const CartesianMapperConfig &config() const { return config_; }

    // Load configuration from a YAML file.
    void load_config(const std::string &yaml_path);

    // Call once at startup to record the home positions.
    //   pika_home: initial Vive pose (used as zero-reference)
    //   ee_home_R, ee_home_p: initial OpenArm EE pose (from FK)
    void set_home(const PikaPose &pika_home, const Eigen::Matrix3d &ee_home_R,
                  const Eigen::Vector3d &ee_home_p);

    bool home_set() const { return home_set_; }

    // Map a Pika pose to desired EE pose and gripper reference.
    //   pika_pose : latest Pika data
    //   current_q : current follower joint angles (for velocity estimation, optional)
    //   R_des_out, p_des_out : desired EE orientation and position
    //   grip_ref_out : desired gripper position [rad]
    void map(const PikaPose &pika_pose, const std::vector<double> &current_q,
             Eigen::Matrix3d &R_des_out, Eigen::Vector3d &p_des_out, double &grip_ref_out);

    // Reset home (e.g., on button press to re-zero)
    void reset_home() {
        home_set_ = false;
        accepted_relative_position_valid_ = false;
        accepted_relative_rot_valid_ = false;
        last_desired_pose_valid_ = false;
    }

private:
    CartesianMapperConfig config_;

    bool home_set_ = false;
    Eigen::Vector3d pika_pos_home_;   // Vive position at startup
    Eigen::Matrix3d pika_rot_home_;   // Vive rotation at startup
    Eigen::Vector3d ee_pos_home_;     // OpenArm EE position at startup
    Eigen::Matrix3d ee_rot_home_;     // OpenArm EE rotation at startup
    Eigen::Vector3d accepted_relative_position_ = Eigen::Vector3d::Zero();
    bool accepted_relative_position_valid_ = false;
    Eigen::Matrix3d accepted_relative_rot_ = Eigen::Matrix3d::Identity();
    bool accepted_relative_rot_valid_ = false;
    Eigen::Vector3d last_desired_position_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d last_desired_rotation_ = Eigen::Matrix3d::Identity();
    bool last_desired_pose_valid_ = false;

    // Previous q_ref for velocity estimation
    std::vector<double> prev_q_ref_;
    bool prev_q_ref_valid_ = false;

    // Gripper continuity filter
    double last_grip_angle_ = 0.0;
    bool last_grip_angle_valid_ = false;
};
