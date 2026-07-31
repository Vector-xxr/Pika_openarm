#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <vector>

// Velocity-level DLS IK (primary Pika teleop path; ported from vr_teleop CartesianController).
struct DampedLeastSquaresConfig {
    bool enabled = true;
    double kp_pos = 4.0;                 // 1/s
    double kp_ori = 2.0;                 // 1/s
    double max_linear_speed_mps = 0.60;  // m/s
    double max_angular_speed_radps = 1.5;
    double lambda = 0.08;                // DLS damping
    double max_joint_velocity_radps = 1.5;
    std::vector<double> max_joint_velocity_radps_vec;
    bool use_nullspace = false;
    double nullspace_gain = 0.2;
    std::vector<double> nullspace_joint_weights;
    // DLS task mobility weights (higher => prefer that joint for Cartesian twist).
    std::vector<double> task_joint_weights;
    double soft_limit_margin_frac = 0.12;
    double max_tracking_error_rad = 0.25;
    double emergency_step_scale = 2.0;
    bool debug_log = false;
    double debug_log_hz = 5.0;
};

enum class OrientationMode {
    HOLD_CLUTCH_ORIENTATION = 0,  // position teleop; TCP rotation held at activation
    VR_ORIENTATION = 1,           // full 6-DOF TCP target tracking
    FREE_ORIENTATION = 2,         // position only; orientation not commanded
};

struct CartesianControllerConfig {
    OrientationMode orientation_mode = OrientationMode::VR_ORIENTATION;
    bool position_only_teleop = false;
    double gripper_open = 0.0;
    double gripper_closed = 0.044;
    double max_gripper_speed = 0.05;
    DampedLeastSquaresConfig dls;

    // Optional HybrIK elbow-swivel hint (cch mode). Primary TCP DLS unchanged.
    bool elbow_hint_enabled = false;
    double elbow_hint_weight = 0.35;
    double elbow_hint_max_delta_rad = 0.7;
    double elbow_hint_sign = 1.0;
    double elbow_hint_offset_rad = 0.0;
    double elbow_hint_slew_radps = 1.2;
    double elbow_hint_timeout_sec = 0.25;
    double elbow_phi_gain = 2.0;       // 1/s toward phi_goal in nullspace
    double elbow_phi_max_rate = 1.5;   // rad/s along self-motion
    double elbow_hint_min_conf = 0.15;
    // TCP-first yield: scale HybrIK weight → 0 as TCP error grows (soft→hard).
    double elbow_tcp_pos_err_soft_m = 0.01;
    double elbow_tcp_pos_err_hard_m = 0.04;
    double elbow_tcp_ori_err_soft_rad = 0.08;
    double elbow_tcp_ori_err_hard_rad = 0.25;
    // Cap task-space leakage induced by the elbow nullspace term.
    double elbow_tcp_leak_lin_mps = 0.02;
    double elbow_tcp_leak_ang_radps = 0.15;
    // Chain segment indices for shoulder / elbow (<0 => auto from chain length).
    int shoulder_segment = -1;
    int elbow_segment = -1;
};

enum class CartesianControlState {
    Idle,
    Active,
    Degraded,
    Hold,
    Estop,
};
