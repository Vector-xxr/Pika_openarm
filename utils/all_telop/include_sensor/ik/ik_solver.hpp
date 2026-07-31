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

#include <controller/cartesian_types.hpp>
#include <controller/dynamics.hpp>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

// Abstract single-shot Cartesian IK step: given a commanded joint-space seed (q_cmd,
// NOT necessarily the measured joints) and a desired TCP pose, produce the joint-space
// segment endpoint (q_end) that the interpolator should track toward next.
class IkSolver {
public:
    virtual ~IkSolver() = default;

    // q_cmd    : joint-space seed used as both the FK query point and the integration
    //            base (typically the interpolator's current commanded position).
    // p_des    : desired TCP position (base/URDF frame).
    // q_des    : desired TCP orientation (base/URDF frame).
    // grip     : desired gripper reference, passed through to grip_out.
    // dt       : time step [s] used to integrate the solved joint velocity.
    // q_end_out: solved joint-space segment endpoint (size = dyn->GetDof()).
    // grip_out : gripper reference to publish alongside q_end_out.
    // Returns false on solve failure; on failure q_end_out is left equal to q_cmd.
    virtual bool step(const std::vector<double>& q_cmd, const Eigen::Vector3d& p_des,
                      const Eigen::Quaterniond& q_des, double grip, double dt,
                      std::vector<double>* q_end_out, double* grip_out) = 0;

    // Reset any internal state (e.g. sign-continuity tracking) using q_cmd as the
    // current joint-space seed.
    virtual void reset(const std::vector<double>& q_cmd) = 0;
};

// Factory: creates an IkSolver by name ("dls" -> DlsIk). Returns nullptr on unknown name.
std::unique_ptr<IkSolver> create_ik_solver(const std::string& name, Dynamics* dyn,
                                           const CartesianControllerConfig& cfg);
