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
#include <mutex>
#include <vector>

// Shared, thread-safe handoff point between the (slower) Cartesian/IK thread and the
// fast joint-space interpolation/control loop. The IK thread solves a new joint-space
// segment endpoint and publishes it here; the control loop consumes it under `mutex`
// and hands it to a JointInterpolator via set_segment().
struct JointSegmentCache {
    std::mutex mutex;
    std::vector<double> q_end;
    double grip = 0;
    uint64_t seq = 0;
    bool valid = false;
};
