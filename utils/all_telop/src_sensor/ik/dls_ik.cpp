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

#include <ik/dls_ik.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

DlsIk::DlsIk(Dynamics* dynamics, CartesianControllerConfig config)
    : dynamics_(dynamics), config_(std::move(config)) {}

Eigen::Quaterniond DlsIk::align_sign(const Eigen::Quaterniond& reference,
                                     Eigen::Quaterniond candidate) {
    if (reference.coeffs().dot(candidate.coeffs()) < 0.0) {
        candidate.coeffs() = -candidate.coeffs();
    }
    return candidate.normalized();
}

void DlsIk::reset(const std::vector<double>& q_cmd) {
    (void)q_cmd;
    last_q_des_valid_ = false;
}

bool DlsIk::step(const std::vector<double>& q_cmd, const Eigen::Vector3d& p_des,
                 const Eigen::Quaterniond& q_des, double grip, double dt,
                 std::vector<double>* q_end_out, double* grip_out) {
    if (grip_out != nullptr) {
        *grip_out = grip;
    }
    if (q_end_out == nullptr) {
        return false;
    }
    // Failure contract: leave q_end as q_cmd.
    *q_end_out = q_cmd;

    if (dynamics_ == nullptr || dt <= 0.0) {
        return false;
    }
    const size_t dof = dynamics_->GetDof();
    if (q_cmd.size() != dof) {
        return false;
    }

    Eigen::Quaterniond q_des_aligned = q_des.normalized();
    if (last_q_des_valid_) {
        q_des_aligned = align_sign(last_q_des_, q_des_aligned);
    }
    last_q_des_ = q_des_aligned;
    last_q_des_valid_ = true;

    Eigen::Matrix3d r_curr = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_curr = Eigen::Vector3d::Zero();
    dynamics_->GetEECordinate(q_cmd.data(), r_curr, p_curr);
    Eigen::Quaterniond q_curr(r_curr);
    q_curr.normalize();

    const Eigen::Vector3d e_pos = p_des - p_curr;

    // Simple quaternion error vector: q_err = q_curr^-1 * q_des, e_ori = 2*vec(q_err).
    Eigen::Quaterniond q_err = q_curr.inverse() * q_des_aligned;
    if (q_err.w() < 0.0) {
        q_err.coeffs() = -q_err.coeffs();
    }
    const Eigen::Vector3d e_ori(2.0 * q_err.x(), 2.0 * q_err.y(), 2.0 * q_err.z());

    Eigen::Vector3d v_lin = config_.dls.kp_pos * e_pos;
    Eigen::Vector3d v_ang = config_.dls.kp_ori * e_ori;

    const double v_norm = v_lin.norm();
    if (config_.dls.max_linear_speed_mps > 0.0 && v_norm > config_.dls.max_linear_speed_mps) {
        v_lin *= config_.dls.max_linear_speed_mps / v_norm;
    }
    const double w_norm = v_ang.norm();
    if (config_.dls.max_angular_speed_radps > 0.0 && w_norm > config_.dls.max_angular_speed_radps) {
        v_ang *= config_.dls.max_angular_speed_radps / w_norm;
    }

    Eigen::VectorXd twist(6);
    twist.head<3>() = v_lin;
    twist.tail<3>() = v_ang;

    std::vector<double> q_min;
    std::vector<double> q_max;
    const bool have_joint_limits =
        dynamics_->GetJointPositionLimits(&q_min, &q_max) && q_min.size() == dof &&
        q_max.size() == dof;
    const std::vector<double>* task_weights =
        config_.dls.task_joint_weights.empty() ? nullptr : &config_.dls.task_joint_weights;
    Eigen::VectorXd q_dot;
    if (!dynamics_->SolveDlsIk(q_cmd.data(), twist, config_.dls.lambda, &q_dot, nullptr,
                               task_weights) ||
        !q_dot.allFinite()) {
        return false;
    }

    std::vector<double> q_end(dof, 0.0);
    for (size_t i = 0; i < dof; ++i) {
        double dq = q_dot[static_cast<Eigen::Index>(i)];
        double vmax = config_.dls.max_joint_velocity_radps;
        if (i < config_.dls.max_joint_velocity_radps_vec.size()) {
            vmax = config_.dls.max_joint_velocity_radps_vec[i];
        }
        if (vmax > 0.0) {
            dq = std::clamp(dq, -vmax, vmax);
        }
        q_end[i] = q_cmd[i] + dq * dt;
    }
    if (have_joint_limits) {
        for (size_t i = 0; i < dof; ++i) {
            q_end[i] = std::clamp(q_end[i], q_min[i], q_max[i]);
        }
    }

    *q_end_out = std::move(q_end);
    return true;
}
