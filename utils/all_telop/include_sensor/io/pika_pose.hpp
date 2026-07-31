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

// PikaPose: parsed from a single CSV line sent by pika_to_openarm_bridge.py
// Format: time,x,y,z,roll,pitch,yaw,qw,qx,qy,qz,gx,gy,gz,grip_angle,grip_distance
struct PikaPose {
    double time = 0.0;
    // Vive optical localization (absolute 6DOF, 120Hz)
    double x = 0.0, y = 0.0, z = 0.0;           // position [m]
    double roll = 0.0, pitch = 0.0, yaw = 0.0;   // Euler angles [rad]
    // 9-axis IMU quaternion and angular velocity (~100Hz)
    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;  // orientation quaternion
    double gx = 0.0, gy = 0.0, gz = 0.0;             // angular velocity [rad/s]
    // Gripper encoder (150Hz)
    double grip_angle = 0.0;       // motor angle [rad], range 0~1.67
    double grip_distance = 0.0;    // gripper opening distance [m]
    bool valid = false;            // true if at least one packet received
};
