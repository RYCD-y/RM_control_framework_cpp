#include "drv_servo.hpp"

#include <cmath>

#include "hal_tim.hpp"

namespace {

bool IsFiniteCalibration(const ServoCalibration& c) {
    return std::isfinite(c.min_angle_rad) && std::isfinite(c.max_angle_rad) &&
           std::isfinite(c.min_pulse_us) && std::isfinite(c.max_pulse_us) &&
           c.max_angle_rad > c.min_angle_rad && c.max_pulse_us > c.min_pulse_us &&
           c.min_pulse_us >= 0.0f;
}

float Clamp(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

}  // namespace

PwmServo::PwmServo(TIM_HandleTypeDef* htim, uint32_t channel, const ServoCalibration& calibration)
    : htim_(htim), channel_(channel), calibration_(calibration) {}

bool PwmServo::IsValidChannel(uint32_t channel) {
    return channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2 || channel == TIM_CHANNEL_3 ||
           channel == TIM_CHANNEL_4;
}

HAL_StatusTypeDef PwmServo::Init() {
    if (htim_ == nullptr || !IsValidChannel(channel_) || !IsFiniteCalibration(calibration_))
        return HAL_ERROR;

    ticks_per_us_ = tim_util::TicksPerUs(htim_);
    if (!(ticks_per_us_ > 0.0f)) return HAL_ERROR;

    // 周期必须能容纳最大脉宽，否则该脉宽无法输出。
    const float period_ticks = static_cast<float>(__HAL_TIM_GET_AUTORELOAD(htim_));
    if (calibration_.max_pulse_us * ticks_per_us_ > period_ticks) return HAL_ERROR;

    target_angle_rad_ = calibration_.min_angle_rad;
    target_pulse_us_ = calibration_.min_pulse_us;
    initialized_ = true;
    return HAL_OK;
}

HAL_StatusTypeDef PwmServo::Enable() {
    if (!initialized_) return HAL_ERROR;
    // 先装载有效占空比再启动，避免启动瞬间输出 0 脉冲。
    WritePulseUs(target_pulse_us_);
    if (HAL_TIM_PWM_Start(htim_, channel_) != HAL_OK) return HAL_ERROR;
    enabled_ = true;
    return HAL_OK;
}

void PwmServo::Disable() {
    if (!initialized_) return;
    (void)HAL_TIM_PWM_Stop(htim_, channel_);
    enabled_ = false;
}

float PwmServo::AngleToPulse(float angle_rad) const {
    const float span = calibration_.max_angle_rad - calibration_.min_angle_rad;
    float ratio = (angle_rad - calibration_.min_angle_rad) / span;
    if (calibration_.reversed) ratio = 1.0f - ratio;
    return calibration_.min_pulse_us +
           ratio * (calibration_.max_pulse_us - calibration_.min_pulse_us);
}

float PwmServo::PulseToAngle(float pulse_us) const {
    const float span = calibration_.max_pulse_us - calibration_.min_pulse_us;
    float ratio = (pulse_us - calibration_.min_pulse_us) / span;
    if (calibration_.reversed) ratio = 1.0f - ratio;
    return calibration_.min_angle_rad +
           ratio * (calibration_.max_angle_rad - calibration_.min_angle_rad);
}

void PwmServo::WritePulseUs(float pulse_us) {
    const float period_ticks = static_cast<float>(__HAL_TIM_GET_AUTORELOAD(htim_));
    const float ticks = Clamp(pulse_us * ticks_per_us_, 0.0f, period_ticks);
    __HAL_TIM_SET_COMPARE(htim_, channel_, static_cast<uint32_t>(ticks + 0.5f));
}

motor::Status PwmServo::SetTargetPositionRad(float position_rad) {
    if (!initialized_) return motor::Status::kNotReady;
    if (!std::isfinite(position_rad)) return motor::Status::kInvalidArgument;

    const float angle = Clamp(position_rad, calibration_.min_angle_rad, calibration_.max_angle_rad);
    target_angle_rad_ = angle;
    target_pulse_us_ = AngleToPulse(angle);
    return motor::Status::kOk;
}

motor::Status PwmServo::SetTargetPulseUs(float pulse_us) {
    if (!initialized_) return motor::Status::kNotReady;
    if (!std::isfinite(pulse_us)) return motor::Status::kInvalidArgument;

    const float pulse = Clamp(pulse_us, calibration_.min_pulse_us, calibration_.max_pulse_us);
    target_pulse_us_ = pulse;
    target_angle_rad_ = PulseToAngle(pulse);
    return motor::Status::kOk;
}

motor::Status PwmServo::SendPosition() {
    if (!initialized_ || !enabled_) return motor::Status::kNotReady;
    WritePulseUs(target_pulse_us_);
    return motor::Status::kOk;
}
