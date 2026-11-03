#include "drv_dm_motor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
constexpr uint8_t kEnableCommand = 0xFCU;
constexpr uint8_t kDisableCommand = 0xFDU;
constexpr uint8_t kClearErrorCommand = 0xFBU;
}  // namespace

DmMotor::DmMotor(CAN_HandleTypeDef* hcan, uint32_t command_id, uint32_t feedback_id,
                 const DmMotorSpec& spec)
    : CanNode(hcan, command_id, feedback_id, canCfg::kIdMaskNone), spec_(spec) {}

uint16_t DmMotor::FloatToUInt(float value, float min, float max, uint8_t bits) {
    if (!(max > min) || bits == 0U || bits > 16U) return 0U;
    const float clamped = std::clamp(value, min, max);
    const uint32_t range = (1UL << bits) - 1UL;
    return static_cast<uint16_t>((clamped - min) * static_cast<float>(range) / (max - min));
}

void DmMotor::PutFloatLe(uint8_t* data, float value) {
    uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    data[0] = static_cast<uint8_t>(bits);
    data[1] = static_cast<uint8_t>(bits >> 8U);
    data[2] = static_cast<uint8_t>(bits >> 16U);
    data[3] = static_cast<uint8_t>(bits >> 24U);
}

HAL_StatusTypeDef DmMotor::SendCommand(const uint8_t* data, uint16_t length) {
    if (data == nullptr || length > 8U) return HAL_ERROR;
    return SendStandard(data, length);
}

HAL_StatusTypeDef DmMotor::OnFrame(const uint8_t* data, uint16_t length) {
    if (data == nullptr || length < 8U) return HAL_ERROR;
    error_code_ = data[0] >> 4U;
    const uint16_t position = static_cast<uint16_t>((data[1] << 8U) | data[2]);
    const uint16_t velocity = static_cast<uint16_t>((data[3] << 4U) | (data[4] >> 4U));
    const uint16_t torque = static_cast<uint16_t>(((data[4] & 0x0FU) << 8U) | data[5]);
    position_rad_ = spec_.position_min_rad + static_cast<float>(position) / 65535.0f *
                                                 (spec_.position_max_rad - spec_.position_min_rad);
    velocity_rad_per_sec_ = spec_.velocity_min_rad_per_sec +
                            static_cast<float>(velocity) / 4095.0f *
                                (spec_.velocity_max_rad_per_sec - spec_.velocity_min_rad_per_sec);
    torque_nm_ = spec_.torque_min_nm +
                 static_cast<float>(torque) / 4095.0f * (spec_.torque_max_nm - spec_.torque_min_nm);
    temperature_c_ = static_cast<int8_t>(data[7]);
    return HAL_OK;
}

motor::Status DmMotor::SetTargetTorqueNm(float torque_nm) {
    if (!std::isfinite(torque_nm) || spec_.torque_max_nm <= spec_.torque_min_nm)
        return motor::Status::kInvalidArgument;
    target_torque_nm_ = std::clamp(torque_nm, spec_.torque_min_nm, spec_.torque_max_nm);
    return motor::Status::kOk;
}

motor::Status DmMotor::SendTorque() { return SendMit({0.0f, 0.0f, 0.0f, 0.0f, target_torque_nm_}); }

motor::Status DmMotor::SendMit(const DmMitCommand& command) {
    if (!std::isfinite(command.position_rad) || !std::isfinite(command.velocity_rad_per_sec) ||
        !std::isfinite(command.kp) || !std::isfinite(command.kd) ||
        !std::isfinite(command.feedforward_torque_nm) ||
        spec_.position_max_rad <= spec_.position_min_rad ||
        spec_.velocity_max_rad_per_sec <= spec_.velocity_min_rad_per_sec ||
        spec_.torque_max_nm <= spec_.torque_min_nm || spec_.kp_max <= 0.0f || spec_.kd_max <= 0.0f)
        return motor::Status::kInvalidArgument;

    const uint16_t p =
        FloatToUInt(command.position_rad, spec_.position_min_rad, spec_.position_max_rad, 16U);
    const uint16_t v = FloatToUInt(command.velocity_rad_per_sec, spec_.velocity_min_rad_per_sec,
                                   spec_.velocity_max_rad_per_sec, 12U);
    const uint16_t kp = FloatToUInt(command.kp, 0.0f, spec_.kp_max, 12U);
    const uint16_t kd = FloatToUInt(command.kd, 0.0f, spec_.kd_max, 12U);
    const uint16_t torque =
        FloatToUInt(command.feedforward_torque_nm, spec_.torque_min_nm, spec_.torque_max_nm, 12U);
    const uint8_t frame[8] = {static_cast<uint8_t>(p >> 8U),
                              static_cast<uint8_t>(p),
                              static_cast<uint8_t>(v >> 4U),
                              static_cast<uint8_t>((v & 0x0FU) << 4U | kp >> 8U),
                              static_cast<uint8_t>(kp),
                              static_cast<uint8_t>(kd >> 4U),
                              static_cast<uint8_t>((kd & 0x0FU) << 4U | torque >> 8U),
                              static_cast<uint8_t>(torque)};
    return SendCommand(frame, sizeof(frame)) == HAL_OK ? motor::Status::kOk
                                                       : motor::Status::kIoError;
}

HAL_StatusTypeDef DmMotor::Enable() {
    const uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, kEnableCommand};
    return SendCommand(frame, sizeof(frame));
}

HAL_StatusTypeDef DmMotor::Disable() {
    const uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, kDisableCommand};
    return SendCommand(frame, sizeof(frame));
}

HAL_StatusTypeDef DmMotor::ClearError() {
    const uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, kClearErrorCommand};
    return SendCommand(frame, sizeof(frame));
}

HAL_StatusTypeDef DmMotor::SetZero(bool permanent) {
    const uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                              0xFFU, 0xFFU, 0xFFU, static_cast<uint8_t>(permanent ? 0xFEU : 0xFCU)};
    return SendCommand(frame, sizeof(frame));
}

motor::Status DmMotor::SendPositionVelocity(float position_rad, float max_velocity_rad_per_sec,
                                            float kd) {
    if (!std::isfinite(position_rad) || !std::isfinite(max_velocity_rad_per_sec) ||
        !std::isfinite(kd) || kd <= 0.0f)
        return motor::Status::kInvalidArgument;
    uint8_t frame[8]{};
    PutFloatLe(frame, position_rad);
    PutFloatLe(frame + 4, max_velocity_rad_per_sec);
    return SendCommand(frame, sizeof(frame)) == HAL_OK ? motor::Status::kOk
                                                       : motor::Status::kIoError;
}

motor::Status DmMotor::SendVelocity(float velocity_rad_per_sec, float torque_limit_nm) {
    if (!std::isfinite(velocity_rad_per_sec) || !std::isfinite(torque_limit_nm))
        return motor::Status::kInvalidArgument;
    uint8_t frame[8]{};
    PutFloatLe(frame, velocity_rad_per_sec);
    PutFloatLe(frame + 4, torque_limit_nm);
    return SendCommand(frame, sizeof(frame)) == HAL_OK ? motor::Status::kOk
                                                       : motor::Status::kIoError;
}
