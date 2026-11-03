#include "motor_demo.hpp"

#include <cmath>

namespace motor_demo {

namespace {

// 返回第一个失败状态，便于示例展示"能力调用可能被拒绝"。
motor::Status FirstFailure(motor::Status current, motor::Status next) {
    return current == motor::Status::kOk && next != motor::Status::kOk ? next : current;
}

}  // namespace

motor::Status DrivePositions(motor::IPositionControl* const* motors, size_t count,
                             float position_rad) {
    if (motors == nullptr || !std::isfinite(position_rad)) return motor::Status::kInvalidArgument;

    motor::Status result = motor::Status::kOk;
    for (size_t i = 0; i < count; ++i) {
        if (motors[i] == nullptr) continue;
        result = FirstFailure(result, motors[i]->SetTargetPositionRad(position_rad));
        result = FirstFailure(result, motors[i]->SendPosition());
    }
    return result;
}

motor::Status DriveTorques(motor::ITorqueControl* const* motors, size_t count, float torque_nm) {
    if (motors == nullptr || !std::isfinite(torque_nm)) return motor::Status::kInvalidArgument;

    motor::Status result = motor::Status::kOk;
    for (size_t i = 0; i < count; ++i) {
        if (motors[i] == nullptr) continue;
        result = FirstFailure(result, motors[i]->SetTargetTorqueNm(torque_nm));
        result = FirstFailure(result, motors[i]->SendTorque());
    }
    return result;
}

motor::Status DriveVelocities(motor::IVelocityControl* const* motors, size_t count,
                              float velocity_rad_per_sec) {
    if (motors == nullptr || !std::isfinite(velocity_rad_per_sec))
        return motor::Status::kInvalidArgument;

    motor::Status result = motor::Status::kOk;
    for (size_t i = 0; i < count; ++i) {
        if (motors[i] == nullptr) continue;
        result = FirstFailure(result, motors[i]->SetTargetVelocityRadPerSec(velocity_rad_per_sec));
        result = FirstFailure(result, motors[i]->SendVelocity());
    }
    return result;
}

}  // namespace motor_demo
