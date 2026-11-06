#include "drv_dm_motor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
constexpr uint8_t kEnableCommand = 0xFCU;
constexpr uint8_t kDisableCommand = 0xFDU;
constexpr uint8_t kClearErrorCommand = 0xFBU;
// 协议的位置/速度帧使用弧度，对外接口使用度，发帧前换算
constexpr float kDegToRad = 0.01745329251994329577f;

// HAL 结果→motor::Status：与计划中的映射表一致
motor::Status ToMotorStatus(HAL_StatusTypeDef status) {
    switch (status) {
        case HAL_OK:
            return motor::Status::kOk;
        case HAL_BUSY:
            return motor::Status::kBusy;
        case HAL_TIMEOUT:
        case HAL_ERROR:
        default:
            return motor::Status::kIoError;
    }
}
}  // namespace

// 构造：只绑定 CAN 节点与量程参数，不碰硬件
DmMotor::DmMotor(CAN_HandleTypeDef* hcan, uint32_t command_id, uint32_t feedback_id,
                 const DmMotorSpec& spec)
    : CanNode(hcan, command_id, feedback_id, canCfg::kIdMaskNone), spec_(spec) {}

// 线性映射到 [0, 2^bits-1]，越界则钳位
uint16_t DmMotor::FloatToUInt(float value, float min, float max, uint8_t bits) {
    if (!(max > min) || bits == 0U || bits > 16U) return 0U;
    const float clamped = std::clamp(value, min, max);
    const uint32_t range = (1UL << bits) - 1UL;
    return static_cast<uint16_t>((clamped - min) * static_cast<float>(range) / (max - min));
}

// 按小端字节序写出 float（达妙位置/速度模式使用）
void DmMotor::PutFloatLe(uint8_t* data, float value) {
    uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    data[0] = static_cast<uint8_t>(bits);
    data[1] = static_cast<uint8_t>(bits >> 8U);
    data[2] = static_cast<uint8_t>(bits >> 16U);
    data[3] = static_cast<uint8_t>(bits >> 24U);
}

// 统一发送出口：经典 CAN 单帧最长 8 字节
HAL_StatusTypeDef DmMotor::SendCommand(const uint8_t* data, uint16_t length) {
    if (data == nullptr || length > 8U) return HAL_ERROR;
    return SendStandard(data, length);
}

// 解析反馈帧：ERR 在高 4 位，位置 16 位，速度/扭矩各 12 位
HAL_StatusTypeDef DmMotor::OnFrame(const uint8_t* data, uint16_t length) {
    if (data == nullptr || length < 8U) return HAL_ERROR;
    error_code_ = data[0] >> 4U;
    const uint16_t position = static_cast<uint16_t>((data[1] << 8U) | data[2]);
    const uint16_t velocity = static_cast<uint16_t>((data[3] << 4U) | (data[4] >> 4U));
    const uint16_t torque = static_cast<uint16_t>(((data[4] & 0x0FU) << 8U) | data[5]);
    position_deg_ = spec_.position_min_deg + static_cast<float>(position) / 65535.0f *
                                                 (spec_.position_max_deg - spec_.position_min_deg);
    velocity_deg_per_sec_ = spec_.velocity_min_deg_per_sec +
                            static_cast<float>(velocity) / 4095.0f *
                                (spec_.velocity_max_deg_per_sec - spec_.velocity_min_deg_per_sec);
    torque_nm_ = spec_.torque_min_nm +
                 static_cast<float>(torque) / 4095.0f * (spec_.torque_max_nm - spec_.torque_min_nm);
    temperature_c_ = static_cast<int8_t>(data[7]);
    return HAL_OK;
}

// 保存目标扭矩并钳位到量程内
motor::Status DmMotor::SetTargetTorqueNm(float torque_nm) {
    if (!std::isfinite(torque_nm) || spec_.torque_max_nm <= spec_.torque_min_nm)
        return motor::Status::kInvalidArgument;
    target_torque_nm_ = std::clamp(torque_nm, spec_.torque_min_nm, spec_.torque_max_nm);
    return motor::Status::kOk;
}

// 纯扭矩输出：MIT 模式下 kp=kd=0，只给前馈力矩
motor::Status DmMotor::SendTorque() { return SendMit({0.0f, 0.0f, 0.0f, 0.0f, target_torque_nm_}); }

// 把五个参数按位拼成 8 字节（位置16 + 速度12 + kp12 + kd12 + 扭矩12）
// 把五个参数按位拼成 8 字节（位置16 + 速度12 + kp12 + kd12 + 扭矩12）
// kp/kd 入参为度制，这里换回协议弧度制后再映射
motor::Status DmMotor::SendMit(const motor::MitCommand& command) {
    if (!std::isfinite(command.position_deg) || !std::isfinite(command.velocity_deg_per_sec) ||
        !std::isfinite(command.kp) || !std::isfinite(command.kd) ||
        !std::isfinite(command.feedforward_torque_nm) ||
        spec_.position_max_deg <= spec_.position_min_deg ||
        spec_.velocity_max_deg_per_sec <= spec_.velocity_min_deg_per_sec ||
        spec_.torque_max_nm <= spec_.torque_min_nm || spec_.kp_max <= 0.0f || spec_.kd_max <= 0.0f)
        return motor::Status::kInvalidArgument;

    const uint16_t p =
        FloatToUInt(command.position_deg, spec_.position_min_deg, spec_.position_max_deg, 16U);
    const uint16_t v = FloatToUInt(command.velocity_deg_per_sec, spec_.velocity_min_deg_per_sec,
                                   spec_.velocity_max_deg_per_sec, 12U);
    const uint16_t kp = FloatToUInt(command.kp / kDegToRad, 0.0f, spec_.kp_max, 12U);
    const uint16_t kd = FloatToUInt(command.kd / kDegToRad, 0.0f, spec_.kd_max, 12U);
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

// 使能：FF...FF FC
motor::Status DmMotor::Enable() {
    const uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, kEnableCommand};
    return ToMotorStatus(SendCommand(frame, sizeof(frame)));
}

// 失能：FF...FF FD
motor::Status DmMotor::Disable() {
    const uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, kDisableCommand};
    return ToMotorStatus(SendCommand(frame, sizeof(frame)));
}

// 清错：FF...FF FB
HAL_StatusTypeDef DmMotor::ClearError() {
    const uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, kClearErrorCommand};
    return SendCommand(frame, sizeof(frame));
}

// 置零：FE 为写入存储，FC 为临时生效
HAL_StatusTypeDef DmMotor::SetZero(bool permanent) {
    const uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                              0xFFU, 0xFFU, 0xFFU, static_cast<uint8_t>(permanent ? 0xFEU : 0xFCU)};
    return SendCommand(frame, sizeof(frame));
}

// 位置速度模式：两个 float 小端，kd 必须为正
motor::Status DmMotor::SendPositionVelocity(float position_deg, float max_velocity_deg_per_sec,
                                            float kd) {
    if (!std::isfinite(position_deg) || !std::isfinite(max_velocity_deg_per_sec) ||
        !std::isfinite(kd) || kd <= 0.0f)
        return motor::Status::kInvalidArgument;
    uint8_t frame[8]{};
    // 协议要求弧度与弧度/秒
    PutFloatLe(frame, position_deg * kDegToRad);
    PutFloatLe(frame + 4, max_velocity_deg_per_sec * kDegToRad);
    return SendCommand(frame, sizeof(frame)) == HAL_OK ? motor::Status::kOk
                                                       : motor::Status::kIoError;
}

// 速度模式：目标速度与扭矩限幅各一个 float 小端
motor::Status DmMotor::SendVelocity(float velocity_deg_per_sec, float torque_limit_nm) {
    if (!std::isfinite(velocity_deg_per_sec) || !std::isfinite(torque_limit_nm))
        return motor::Status::kInvalidArgument;
    uint8_t frame[8]{};
    // 协议要求弧度/秒
    PutFloatLe(frame, velocity_deg_per_sec * kDegToRad);
    PutFloatLe(frame + 4, torque_limit_nm);
    return SendCommand(frame, sizeof(frame)) == HAL_OK ? motor::Status::kOk
                                                       : motor::Status::kIoError;
}
