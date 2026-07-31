#pragma once
#include "gripper_serial_io.h"
#include <memory>

// 夹爪控制：向 gripper 串口发 EFFORT / ENABLE / VELOCITY / POSITION / DISABLE
class GripperController {
public:
    explicit GripperController(std::shared_ptr<GripperSerialIO> io);

    void initEffort(float effort_mA);
    void setVelocity(float velocity);
    void enable();
    void disable();
    void sendPosition(float angle_rad, bool mit_mode);

private:
    std::shared_ptr<GripperSerialIO> io_;
};
