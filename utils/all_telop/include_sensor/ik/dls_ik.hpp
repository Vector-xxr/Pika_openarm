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

#include <ik/ik_solver.hpp>

#include <Eigen/Dense>
#include <vector>

// Velocity-level Damped Least Squares (DLS) Cartesian IK step.
// step() evaluates FK/Jacobian at q_cmd (not the measured joints), solves a damped
// least-squares joint velocity via Dynamics::SolveDlsIk, and integrates
// q_end = q_cmd + qdot * dt, clamped to the dynamics joint limits when available.
class DlsIk : public IkSolver {
public:
    DlsIk(Dynamics* dynamics, CartesianControllerConfig config);

    bool step(const std::vector<double>& q_cmd, const Eigen::Vector3d& p_des,
              const Eigen::Quaterniond& q_des, double grip, double dt,
              std::vector<double>* q_end_out, double* grip_out) override;

    void reset(const std::vector<double>& q_cmd) override;

private:
    Dynamics* dynamics_ = nullptr;
    CartesianControllerConfig config_;

    // Tracks the previously-commanded desired orientation so that the double-cover
    // quaternion sign can be kept continuous across successive step() calls.
    Eigen::Quaterniond last_q_des_ = Eigen::Quaterniond::Identity();
    bool last_q_des_valid_ = false;

    static Eigen::Quaterniond align_sign(const Eigen::Quaterniond& reference,
                                         Eigen::Quaterniond candidate);
};
