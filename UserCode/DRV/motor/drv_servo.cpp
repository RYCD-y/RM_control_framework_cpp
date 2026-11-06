#include "drv_servo.hpp"

#include <cmath>

#include "hal_tim.hpp"

namespace {

// 校验标定合法：范围递增且有限
bool IsFiniteCalibration(const ServoCalibration& c) {
    return std::isfinite(c.min_angle_deg) && std::isfinite(c.max_angle_deg) &&
           std::isfinite(c.min_pulse_us) && std::isfinite(c.max_pulse_us) &&
           c.max_angle_deg > c.min_angle_deg && c.max_pulse_us > c.min_pulse_us &&
           c.min_pulse_us >= 0.0f;
}

// 把 value 限幅到 [low, high]
float Clamp(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

}  // namespace

// 构造：只保存硬件绑定与标定
PwmServo::PwmServo(TIM_HandleTypeDef* htim, uint32_t channel, const ServoCalibration& calibration)
    : htim_(htim), channel_(channel), calibration_(calibration) {}

// 只允许四路正向 PWM 通道
bool PwmServo::IsValidChannel(uint32_t channel) {
    return channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2 || channel == TIM_CHANNEL_3 ||
           channel == TIM_CHANNEL_4;
}

// 校验标定与 PWM 周期，算出计数值比例；不启动输出
HAL_StatusTypeDef PwmServo::Init() {
    if (htim_ == nullptr || !IsValidChannel(channel_) || !IsFiniteCalibration(calibration_))
        return HAL_ERROR;

    ticks_per_us_ = tim_util::TicksPerUs(htim_);
    if (!(ticks_per_us_ > 0.0f)) return HAL_ERROR;

    // 周期必须能容纳最大脉宽，否则该脉宽无法输出。
    const float period_ticks = static_cast<float>(__HAL_TIM_GET_AUTORELOAD(htim_));
    if (calibration_.max_pulse_us * ticks_per_us_ > period_ticks) return HAL_ERROR;

    target_angle_deg_ = calibration_.min_angle_deg;
    target_pulse_us_ = calibration_.min_pulse_us;
    initialized_ = true;
    return HAL_OK;
}

// 先写占空比再启动 PWM，避免启动瞬间输出零脉冲
motor::Status PwmServo::Enable() {
    if (!initialized_) return motor::Status::kNotReady;
    // 先装载有效占空比再启动，避免启动瞬间输出 0 脉冲。
    WritePulseUs(target_pulse_us_);
    if (HAL_TIM_PWM_Start(htim_, channel_) != HAL_OK) return motor::Status::kIoError;
    enabled_ = true;
    return motor::Status::kOk;
}

// 停止 PWM，舵机失去保持力矩
motor::Status PwmServo::Disable() {
    if (!initialized_) return motor::Status::kNotReady;
    (void)HAL_TIM_PWM_Stop(htim_, channel_);
    enabled_ = false;
    return motor::Status::kOk;
}

// 接口实现：失能（未初始化时返回 kNotReady）

// 角度→脉宽：按标定线性映射，reversed 时翻转方向
float PwmServo::AngleToPulse(float angle_deg) const {
    const float span = calibration_.max_angle_deg - calibration_.min_angle_deg;
    float ratio = (angle_deg - calibration_.min_angle_deg) / span;
    if (calibration_.reversed) ratio = 1.0f - ratio;
    return calibration_.min_pulse_us +
           ratio * (calibration_.max_pulse_us - calibration_.min_pulse_us);
}

// 脉宽→角度：同标定反向映射
float PwmServo::PulseToAngle(float pulse_us) const {
    const float span = calibration_.max_pulse_us - calibration_.min_pulse_us;
    float ratio = (pulse_us - calibration_.min_pulse_us) / span;
    if (calibration_.reversed) ratio = 1.0f - ratio;
    return calibration_.min_angle_deg +
           ratio * (calibration_.max_angle_deg - calibration_.min_angle_deg);
}

// 脉宽换算计数值，限幅在 [0, ARR] 后写 CCR
void PwmServo::WritePulseUs(float pulse_us) {
    const float period_ticks = static_cast<float>(__HAL_TIM_GET_AUTORELOAD(htim_));
    const float ticks = Clamp(pulse_us * ticks_per_us_, 0.0f, period_ticks);
    __HAL_TIM_SET_COMPARE(htim_, channel_, static_cast<uint32_t>(ticks + 0.5f));
}

// 保存目标角度并钳位，只表示已接受
motor::Status PwmServo::SetTargetPositionDeg(float position_deg) {
    if (!initialized_) return motor::Status::kNotReady;
    if (!std::isfinite(position_deg)) return motor::Status::kInvalidArgument;

    const float angle = Clamp(position_deg, calibration_.min_angle_deg, calibration_.max_angle_deg);
    target_angle_deg_ = angle;
    target_pulse_us_ = AngleToPulse(angle);
    return motor::Status::kOk;
}

// 直接设定目标脉宽，同时反算角度保持一致
motor::Status PwmServo::SetTargetPulseUs(float pulse_us) {
    if (!initialized_) return motor::Status::kNotReady;
    if (!std::isfinite(pulse_us)) return motor::Status::kInvalidArgument;

    const float pulse = Clamp(pulse_us, calibration_.min_pulse_us, calibration_.max_pulse_us);
    target_pulse_us_ = pulse;
    target_angle_deg_ = PulseToAngle(pulse);
    return motor::Status::kOk;
}

// 仅在已初始化且已使能时真正写 CCR
motor::Status PwmServo::SendPosition() {
    if (!initialized_ || !enabled_) return motor::Status::kNotReady;
    WritePulseUs(target_pulse_us_);
    return motor::Status::kOk;
}
