#ifndef THREE_FRICTION_VESC_PROTOCOL_HPP
#define THREE_FRICTION_VESC_PROTOCOL_HPP

#include <cmath>
#include <cstdint>

// VESC CAN 协议的纯换算与打包，不依赖 HAL，便于主机测试。
namespace vesc {

// 反馈帧 ext_id 前缀：0x900 | controller_id
inline constexpr uint32_t kStatusBaseId = 0x900U;

// 大端写出 int32（VESC 多字节字段均为高字节在前）
inline void PutBeI32(uint8_t* data, int32_t value) {
    const uint32_t raw = static_cast<uint32_t>(value);
    data[0] = static_cast<uint8_t>(raw >> 24U);
    data[1] = static_cast<uint8_t>(raw >> 16U);
    data[2] = static_cast<uint8_t>(raw >> 8U);
    data[3] = static_cast<uint8_t>(raw);
}

// 大端读入 int32
inline int32_t GetBeI32(const uint8_t* data) {
    return static_cast<int32_t>((static_cast<uint32_t>(data[0]) << 24U) |
                                (static_cast<uint32_t>(data[1]) << 16U) |
                                (static_cast<uint32_t>(data[2]) << 8U) |
                                static_cast<uint32_t>(data[3]));
}

// 大端读入 int16
inline int16_t GetBeI16(const uint8_t* data) {
    return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8U) |
                                static_cast<uint16_t>(data[1]));
}

inline float Clamp(float value, float limit) {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

// 占空比 -1..1 → raw（×1e5）
inline int32_t DutyToRaw(float duty) {
    return static_cast<int32_t>(Clamp(duty, 1.0f) * 100000.0f);
}

// 电流(A) → raw（×1e3）
inline int32_t CurrentToRaw(float amp, float limit) {
    return static_cast<int32_t>(Clamp(amp, limit) * 1000.0f);
}

// 转子角度(度) → raw（×1e6）
inline int32_t RotorDegToRaw(float rotor_deg) {
    return static_cast<int32_t>(rotor_deg * 1000000.0f);
}

// ERPM → 输出轴 RPM：÷极对数 → 机械 RPM → ÷减速比
inline float ErpmToRpm(int32_t erpm, uint8_t pole_pairs, float gear_ratio) {
    if (pole_pairs == 0U || gear_ratio <= 0.0f) return 0.0f;
    return static_cast<float>(erpm) / static_cast<float>(pole_pairs) / gear_ratio;
}

// 输出轴 RPM → ERPM（ErpmToRpm 的逆运算）
inline int32_t RpmToErpm(float rpm, uint8_t pole_pairs, float gear_ratio) {
    return static_cast<int32_t>(rpm * gear_ratio * static_cast<float>(pole_pairs));
}

// 输出轴扭矩(N·m) → 电流(A)
inline float TorqueToCurrent(float torque_nm, float kt_output_nm_per_a) {
    if (kt_output_nm_per_a <= 0.0f) return 0.0f;
    return torque_nm / kt_output_nm_per_a;
}

}  // namespace vesc

#endif  // THREE_FRICTION_VESC_PROTOCOL_HPP
