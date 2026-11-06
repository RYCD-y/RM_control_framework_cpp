#ifndef THREE_FRICTION_DRV_SERVO_HPP
#define THREE_FRICTION_DRV_SERVO_HPP

#include <cstdint>

#include "motor_base.hpp"
#include "stm32f4xx_hal.h"

// 角度/脉宽标定：min_angle 对应 min_pulse，max_angle 对应 max_pulse。
// 单位：角度为度（deg），脉宽为微秒（us）。
struct ServoCalibration {
    float min_angle_deg = 0.0f;
    float max_angle_deg = 180.0f;
    float min_pulse_us = 500.0f;
    float max_pulse_us = 2500.0f;
    bool reversed = false;
};

// 参考预设：0~180 度、500~2500 us、50 Hz。来自通用 PWM 舵机示例，
// 不代表任何具体厂家的出厂规格，上机前必须按实物校准安全端点。
inline constexpr ServoCalibration kReferenceServoCalibration{};

class PwmServo final : public motor::IPositionControl, public motor::IEnableControl {
   public:
    // 构造：注入定时器/通道与标定，不碰硬件
    PwmServo(TIM_HandleTypeDef* htim, uint32_t channel, const ServoCalibration& calibration);

    // 只校验并准备，不启动 PWM。
    HAL_StatusTypeDef Init();
    // 显式启动输出：先装载有效 CCR，再启动 PWM，避免上电瞬间输出零脉冲。
    motor::Status Enable() override;
    // 关闭 PWM 输出，舵机失去保持力矩
    motor::Status Disable() override;

    // 保存目标角度（度）并钳位到标定范围
    motor::Status SetTargetPositionDeg(float position_deg) override;
    // 把当前目标写入比较寄存器（需已 Enable）
    motor::Status SendPosition() override;

    // 直接按脉宽设定（会被钳位到标定范围）。仅影响目标，不启动 PWM。
    motor::Status SetTargetPulseUs(float pulse_us);

    // 只读：目标角度（度）/目标脉宽/使能状态/初始化状态
    float TargetPositionDeg() const { return target_angle_deg_; }
    float TargetPulseUs() const { return target_pulse_us_; }
    bool IsEnabled() const { return enabled_; }
    bool IsInitialized() const { return initialized_; }

   private:
    // 只接受 TIM_CHANNEL_1..4
    static bool IsValidChannel(uint32_t channel);
    // 角度（度）→脉宽（含反向）
    float AngleToPulse(float angle_deg) const;
    // 脉宽→角度（度，含反向）
    float PulseToAngle(float pulse_us) const;
    // 脉宽换算为计数值并限幅后写入 CCR
    void WritePulseUs(float pulse_us);

    TIM_HandleTypeDef* htim_ = nullptr;
    uint32_t channel_ = 0U;
    ServoCalibration calibration_{};
    float ticks_per_us_ = 0.0f;
    float target_angle_deg_ = 0.0f;
    float target_pulse_us_ = 0.0f;
    bool initialized_ = false;
    bool enabled_ = false;
};

#endif  // THREE_FRICTION_DRV_SERVO_HPP
