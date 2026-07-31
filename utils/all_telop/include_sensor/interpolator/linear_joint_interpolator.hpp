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

#include <interpolator/joint_interpolator.hpp>

// Counting-steps linear interpolator: divides the remaining distance to the segment
// endpoint evenly across the remaining steps on every tick, so it converges exactly on
// q_seg_end after steps_left ticks regardless of intermediate re-segmenting.
class LinearJointInterpolator : public JointInterpolator {
public:
    explicit LinearJointInterpolator(int default_steps = 1);

    void set_segment(const std::vector<double>& q_end, int steps_hint = 0) override;
    void step(double dt, std::vector<double>* q_cmd, std::vector<double>* dq_cmd) override;
    void hold_at(const std::vector<double>& q) override;
    void reset(const std::vector<double>& q_cmd0) override;

private:
    int default_steps_ = 1;
    bool initialized_ = false;
    std::vector<double> q_cmd_;
    std::vector<double> q_seg_start_;
    std::vector<double> q_seg_end_;
    int steps_left_ = 0;
};
