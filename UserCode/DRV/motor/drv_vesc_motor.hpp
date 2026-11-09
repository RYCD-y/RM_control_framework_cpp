#ifndef THREE_FRICTION_DRV_VESC_MOTOR_HPP
#define THREE_FRICTION_DRV_VESC_MOTOR_HPP

#include <cstdint>

#include "hal_can.hpp"
#include "motor_base.hpp"
#include "vesc_protocol.hpp"

// VESC CAN 命令（扩展帧：ext_id = (command << 8) | controller_id）
enum class VescPacket : uint8_t {
    kSetDuty = 0,
    kSetCurrent = 1,
    kSetCurrentBrake = 2,
    kSetRpm = 3,
    kSetPos = 4,
    kStatus1 = 9,
};

// 反馈帧固定 ID 前缀：0x900 | controller_id
// 反馈帧 ID 前缀见 vesc::kStatusBaseId

struct VescSpec {
    uint8_t pole_pairs;        // 极对数（ERPM = 机械 RPM × 极对数）
    float gear_ratio;          // 转子 → 输出轴
    float kt_output_nm_per_a;  // 输出轴转矩常数 N·m/A（手册值）
    float current_max_a;       // 电流限幅
    uint32_t erpm_max;         // 电转速限幅
};

// 预设取自本地 RoboMaster 手册：
//   M3508：极对数 7，减速比 3591/187，转矩常数 0.3 N·m/A（输出轴）
//   M2006：极对数 7，减速比 36:1，转矩常数 0.18 N·m/A（输出轴）
// 电流限幅沿用 C620（20 A）与 C610（10 A）量程；ERPM 上限按额定转速折算，
// 上机前需按实际电机与 VESC 配置校准。
inline constexpr VescSpec kVescM3508{7U, 3591.0f / 187.0f, 0.3f, 20.0f, 63000U};
inline constexpr VescSpec kVescM2006{7U, 36.0f, 0.18f, 10.0f, 104800U};

class VescMotor final : public motor::ITorqueControl,
                        public motor::IVelocityControl,
                        public motor::IPositionControl,
                        public CanNode {
   public:
    // controller_id 对应 VESC Tool 里的 VESC ID
    VescMotor(CAN_HandleTypeDef* hcan, uint8_t controller_id, const VescSpec& spec);

    // 解析 STATUS_1：ERPM、电流(÷10)、duty(÷1000)
    HAL_StatusTypeDef OnFrame(const uint8_t* data, uint16_t len) override;

    // 扭矩：目标扭矩 ÷ 转矩常数 = 电流 → SET_CURRENT
    motor::Status SetTargetTorqueNm(float torque_nm) override;
    motor::Status SendTorque() override;

    // 速度：输出轴 RPM → 转子 RPM → ERPM → SET_RPM
    motor::Status SetTargetVelocityRpm(float velocity_rpm) override;
    motor::Status SendVelocity() override;

    // 位置：输出轴度 → 转子度 → SET_POS（协议无位置反馈，只下发）
    motor::Status SetTargetPositionDeg(float position_deg) override;
    motor::Status SendPosition() override;

    // 专有命令：占空比与制动电流
    motor::Status SetDuty(float duty);
    motor::Status SetBrakeCurrent(float amp);

    // STATUS_1 反馈
    float VelocityRpm() const { return velocity_rpm_; }
    float CurrentA() const { return current_a_; }
    float Duty() const { return duty_; }

   private:
    // 按命令号与大端 int32 载荷发送一帧
    motor::Status SendPacket(VescPacket packet, int32_t raw);

    VescSpec spec_;
    uint8_t controller_id_;
    float target_torque_nm_ = 0.0f;
    float target_velocity_rpm_ = 0.0f;
    float target_position_deg_ = 0.0f;
    float velocity_rpm_ = 0.0f;
    float current_a_ = 0.0f;
    float duty_ = 0.0f;
};

#endif  // THREE_FRICTION_DRV_VESC_MOTOR_HPP
