#ifndef THREE_FRICTION_DRV_SERVO_HPP
#define THREE_FRICTION_DRV_SERVO_HPP

#include <cstdint>

#include "motor_base.hpp"
#include "stm32f4xx_hal.h"

// 角度/脉宽标定：min_angle 对应 min_pulse，max_angle 对应 max_pulse。
// 单位：角度 rad，脉宽 us。
struct ServoCalibration {
    float min_angle_rad = 0.0f;
    float max_angle_rad = 3.14159265358979323846f;
    float min_pulse_us = 500.0f;
    float max_pulse_us = 2500.0f;
    bool reversed = false;
};

// 参考预设：0~π rad、500~2500 us、50 Hz。来自通用 PWM 舵机示例，
// 不代表任何具体厂家的出厂规格，上机前必须按实物校准安全端点。
inline constexpr ServoCalibration kReferenceServoCalibration{};

class PwmServo final : public motor::IPositionControl {
   public:
    PwmServo(TIM_HandleTypeDef* htim, uint32_t channel, const ServoCalibration& calibration);

    // 只校验并准备，不启动 PWM。
    HAL_StatusTypeDef Init();
    // 显式启动输出：先装载有效 CCR，再启动 PWM，避免上电瞬间输出零脉冲。
    HAL_StatusTypeDef Enable();
    void Disable();

    motor::Status SetTargetPositionRad(float position_rad) override;
    motor::Status SendPosition() override;

    // 直接按脉宽设定（会被钳位到标定范围）。仅影响目标，不启动 PWM。
    motor::Status SetTargetPulseUs(float pulse_us);

    float TargetPositionRad() const { return target_angle_rad_; }
    float TargetPulseUs() const { return target_pulse_us_; }
    bool IsEnabled() const { return enabled_; }
    bool IsInitialized() const { return initialized_; }

   private:
    static bool IsValidChannel(uint32_t channel);
    float AngleToPulse(float angle_rad) const;
    float PulseToAngle(float pulse_us) const;
    void WritePulseUs(float pulse_us);

    TIM_HandleTypeDef* htim_ = nullptr;
    uint32_t channel_ = 0U;
    ServoCalibration calibration_{};
    float ticks_per_us_ = 0.0f;
    float target_angle_rad_ = 0.0f;
    float target_pulse_us_ = 0.0f;
    bool initialized_ = false;
    bool enabled_ = false;
};

#endif  // THREE_FRICTION_DRV_SERVO_HPP
