#include "drv_vesc_motor.hpp"

#include <cmath>

#include "vesc_protocol.hpp"

// 构造：发送 ID 无固定值（每条命令自拼），反馈 ID 为 0x900|id，掩码用扩展帧全匹配
VescMotor::VescMotor(CAN_HandleTypeDef* hcan, uint8_t controller_id, const VescSpec& spec)
    : CanNode(hcan, controller_id, vesc::kStatusBaseId | controller_id, canCfg::kExtIdMaskFull),
      spec_(spec),
      controller_id_(controller_id) {}

// 按命令号发送一帧：ext_id = (command << 8) | controller_id
motor::Status VescMotor::SendPacket(VescPacket packet, int32_t raw) {
    uint8_t frame[4] = {};
    vesc::PutBeI32(frame, raw);
    const uint32_t ext_id = (static_cast<uint32_t>(packet) << 8U) |
                            static_cast<uint32_t>(controller_id_);
    switch (SendExtendedWithId(ext_id, frame, sizeof(frame))) {
        case HAL_OK:
            return motor::Status::kOk;
        case HAL_BUSY:
            return motor::Status::kBusy;
        default:
            return motor::Status::kIoError;
    }
}

// STATUS_1：ERPM / 电流÷10 / duty÷1000
HAL_StatusTypeDef VescMotor::OnFrame(const uint8_t* data, uint16_t len) {
    if (data == nullptr || len < 8U) return HAL_ERROR;

    velocity_rpm_ =
        vesc::ErpmToRpm(vesc::GetBeI32(&data[0]), spec_.pole_pairs, spec_.gear_ratio);
    current_a_ = static_cast<float>(vesc::GetBeI16(&data[4])) / 10.0f;
    duty_ = static_cast<float>(vesc::GetBeI16(&data[6])) / 1000.0f;
    return HAL_OK;
}

// 保存目标扭矩；下发时再换算为电流
motor::Status VescMotor::SetTargetTorqueNm(float torque_nm) {
    if (!std::isfinite(torque_nm) || spec_.kt_output_nm_per_a <= 0.0f)
        return motor::Status::kInvalidArgument;
    target_torque_nm_ = torque_nm;
    return motor::Status::kOk;
}

motor::Status VescMotor::SendTorque() {
    if (!std::isfinite(target_torque_nm_) || spec_.kt_output_nm_per_a <= 0.0f)
        return motor::Status::kInvalidArgument;

    const float amp = vesc::TorqueToCurrent(target_torque_nm_, spec_.kt_output_nm_per_a);
    return SendPacket(VescPacket::kSetCurrent, vesc::CurrentToRaw(amp, spec_.current_max_a));
}

// 保存目标速度；下发时换算为 ERPM
motor::Status VescMotor::SetTargetVelocityRpm(float velocity_rpm) {
    if (!std::isfinite(velocity_rpm)) return motor::Status::kInvalidArgument;
    target_velocity_rpm_ = velocity_rpm;
    return motor::Status::kOk;
}

motor::Status VescMotor::SendVelocity() {
    int32_t erpm =
        vesc::RpmToErpm(target_velocity_rpm_, spec_.pole_pairs, spec_.gear_ratio);
    const int32_t limit = static_cast<int32_t>(spec_.erpm_max);
    if (erpm > limit) erpm = limit;
    if (erpm < -limit) erpm = -limit;
    return SendPacket(VescPacket::kSetRpm, erpm);
}

// 保存目标位置（输出轴度）；下发时换算为转子度
motor::Status VescMotor::SetTargetPositionDeg(float position_deg) {
    if (!std::isfinite(position_deg)) return motor::Status::kInvalidArgument;
    target_position_deg_ = position_deg;
    return motor::Status::kOk;
}

motor::Status VescMotor::SendPosition() {
    return SendPacket(VescPacket::kSetPos,
                      vesc::RotorDegToRaw(target_position_deg_ * spec_.gear_ratio));
}

// 占空比 -1..1
motor::Status VescMotor::SetDuty(float duty) {
    if (!std::isfinite(duty)) return motor::Status::kInvalidArgument;
    return SendPacket(VescPacket::kSetDuty, vesc::DutyToRaw(duty));
}

// 制动电流
motor::Status VescMotor::SetBrakeCurrent(float amp) {
    if (!std::isfinite(amp)) return motor::Status::kInvalidArgument;
    return SendPacket(VescPacket::kSetCurrentBrake, vesc::CurrentToRaw(amp, spec_.current_max_a));
}
