#include "gripper_controller.h"
#include "gripper_protocol.h"

GripperController::GripperController(std::shared_ptr<GripperSerialIO> io) : io_(std::move(io)) {}

void GripperController::initEffort(float effort_mA) {
    const float effort = effort_mA / 1000.0f;
    io_->writeBytes(createBinaryCommand<float>(
        static_cast<uint8_t>(GripperSendFlag::EFFORT_CTRL), {effort}));
}

void GripperController::setVelocity(float velocity) {
    if (velocity == 0.0f) return;
    io_->writeBytes(createBinaryCommand<float>(
        static_cast<uint8_t>(GripperSendFlag::VELOCITY_CTRL), {velocity, velocity}));
}

void GripperController::enable() {
    io_->writeBytes(createBinaryCommand<float>(
        static_cast<uint8_t>(GripperSendFlag::ENABLE), {0.0f}));
}

void GripperController::disable() {
    io_->writeBytes(createBinaryCommand<float>(
        static_cast<uint8_t>(GripperSendFlag::DISABLE), {0.0f}));
}

void GripperController::sendPosition(float angle_rad, bool mit_mode) {
    if (angle_rad < 0.0f) angle_rad = 0.0f;
    else if (angle_rad > 1.67f) angle_rad = 1.67f;
    const uint8_t cmd = mit_mode
        ? static_cast<uint8_t>(GripperSendFlag::POSITION_CTRL_MIT)
        : static_cast<uint8_t>(GripperSendFlag::POSITION_CTRL_POS_VEL);
    io_->writeBytes(createBinaryCommand<float>(cmd, {angle_rad}));
}
