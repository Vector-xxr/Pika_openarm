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

#include <interpolator/linear_joint_interpolator.hpp>

#include <algorithm>

LinearJointInterpolator::LinearJointInterpolator(int default_steps)
    : default_steps_(std::max(1, default_steps)) {}

void LinearJointInterpolator::set_segment(const std::vector<double>& q_end, int steps_hint) {
    if (!initialized_ || q_cmd_.size() != q_end.size()) {
        // First segment (or a DOF-count change): start exactly at q_end so we don't
        // introduce a spurious jump from a stale/empty q_cmd_.
        q_cmd_ = q_end;
        initialized_ = true;
    }
    q_seg_start_ = q_cmd_;
    q_seg_end_ = q_end;
    steps_left_ = steps_hint > 0 ? steps_hint : default_steps_;
}

void LinearJointInterpolator::step(double dt, std::vector<double>* q_cmd,
                                   std::vector<double>* dq_cmd) {
    const size_t n = q_cmd_.size();
    if (!initialized_ || n == 0 || q_seg_end_.size() != n) {
        if (q_cmd != nullptr) {
            *q_cmd = q_cmd_;
        }
        if (dq_cmd != nullptr) {
            dq_cmd->assign(n, 0.0);
        }
        return;
    }

    std::vector<double> dq(n, 0.0);
    if (steps_left_ > 0) {
        const double inv_steps = 1.0 / static_cast<double>(steps_left_);
        for (size_t i = 0; i < n; ++i) {
            const double step_i = (q_seg_end_[i] - q_cmd_[i]) * inv_steps;
            q_cmd_[i] += step_i;
            dq[i] = dt > 0.0 ? step_i / dt : 0.0;
        }
        --steps_left_;
    } else {
        // Segment exhausted: zero-order hold at the segment endpoint, zero velocity.
        q_cmd_ = q_seg_end_;
    }

    if (q_cmd != nullptr) {
        *q_cmd = q_cmd_;
    }
    if (dq_cmd != nullptr) {
        *dq_cmd = dq;
    }
}

void LinearJointInterpolator::hold_at(const std::vector<double>& q) {
    q_cmd_ = q;
    q_seg_start_ = q;
    q_seg_end_ = q;
    steps_left_ = 0;
    initialized_ = true;
}

void LinearJointInterpolator::reset(const std::vector<double>& q_cmd0) {
    q_cmd_ = q_cmd0;
    q_seg_start_ = q_cmd0;
    q_seg_end_ = q_cmd0;
    steps_left_ = 0;
    initialized_ = true;
}
