#include "motor_demo.hpp"

#include <cmath>

namespace motor_demo {

namespace {

// 返回第一个失败状态，便于示例展示"能力调用可能被拒绝"。
motor::Status FirstFailure(motor::Status current, motor::Status next) {
    return current == motor::Status::kOk && next != motor::Status::kOk ? next : current;
}

}  // namespace

// 对一组位置能力电机下发同一目标位置并提交；ZDT 与舵机可直接共用
motor::Status DrivePositions(motor::IPositionControl* const* motors, size_t count,
                             float position_deg) {
    if (motors == nullptr || !std::isfinite(position_deg)) return motor::Status::kInvalidArgument;

    motor::Status result = motor::Status::kOk;
    for (size_t i = 0; i < count; ++i) {
        if (motors[i] == nullptr) continue;
        result = FirstFailure(result, motors[i]->SetTargetPositionDeg(position_deg));
        result = FirstFailure(result, motors[i]->SendPosition());
    }
    return result;
}

// 对一组扭矩能力电机下发同一目标扭矩并提交；DJI 与达妙可直接共用
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

// 对一组速度能力电机下发同一目标速度并提交
motor::Status DriveVelocities(motor::IVelocityControl* const* motors, size_t count,
                              float velocity_deg_per_sec) {
    if (motors == nullptr || !std::isfinite(velocity_deg_per_sec))
        return motor::Status::kInvalidArgument;

    motor::Status result = motor::Status::kOk;
    for (size_t i = 0; i < count; ++i) {
        if (motors[i] == nullptr) continue;
        result = FirstFailure(result, motors[i]->SetTargetVelocityDegPerSec(velocity_deg_per_sec));
        result = FirstFailure(result, motors[i]->SendVelocity());
    }
    return result;
}

// 统一使能/失能：enabled=true 逐个使能，否则逐个失能
motor::Status SetEnabled(motor::IEnableControl* const* motors, size_t count, bool enabled) {
    if (motors == nullptr) return motor::Status::kInvalidArgument;

    motor::Status result = motor::Status::kOk;
    for (size_t i = 0; i < count; ++i) {
        if (motors[i] == nullptr) continue;
        result = FirstFailure(result, enabled ? motors[i]->Enable() : motors[i]->Disable());
    }
    return result;
}

// 给一组 MIT 能力电机下发同一命令
motor::Status SendMitToAll(motor::IMitControl* const* motors, size_t count,
                           const motor::MitCommand& command) {
    if (motors == nullptr) return motor::Status::kInvalidArgument;

    motor::Status result = motor::Status::kOk;
    for (size_t i = 0; i < count; ++i) {
        if (motors[i] == nullptr) continue;
        result = FirstFailure(result, motors[i]->SendMit(command));
    }
    return result;
}

}  // namespace motor_demo
