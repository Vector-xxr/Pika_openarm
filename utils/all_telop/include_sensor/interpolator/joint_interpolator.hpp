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

#include <memory>
#include <string>
#include <vector>

// Interpolates joint-space commands between successive IK-solved segment endpoints so
// the fast control loop always has a smooth, per-tick command even though new segment
// endpoints may arrive at a slower, irregular rate.
class JointInterpolator {
public:
    virtual ~JointInterpolator() = default;

    // Start a new segment from the interpolator's current commanded position toward
    // q_end. steps_hint <= 0 uses the interpolator's default step count.
    virtual void set_segment(const std::vector<double>& q_end, int steps_hint = 0) = 0;

    // Advance the interpolator by dt, writing the next commanded position/velocity.
    virtual void step(double dt, std::vector<double>* q_cmd, std::vector<double>* dq_cmd) = 0;

    // Freeze the interpolator at q: zero-order hold with zero commanded velocity.
    virtual void hold_at(const std::vector<double>& q) = 0;

    // Reset internal state to q_cmd0 with no active segment.
    virtual void reset(const std::vector<double>& q_cmd0) = 0;
};

// Factory: creates a JointInterpolator by name ("linear" -> LinearJointInterpolator).
// Returns nullptr on unknown name.
std::unique_ptr<JointInterpolator> create_joint_interpolator(const std::string& name,
                                                              int default_steps);
