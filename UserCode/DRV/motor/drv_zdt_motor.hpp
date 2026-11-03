#ifndef THREE_FRICTION_DRV_ZDT_MOTOR_HPP
#define THREE_FRICTION_DRV_ZDT_MOTOR_HPP

#include <cstddef>
#include <cstdint>

#include "hal_uart.hpp"
#include "motor_base.hpp"

struct ZdtMotorConfig {
    uint16_t pulses_per_revolution = 3200U;
    uint16_t default_velocity_rpm = 1000U;
    uint8_t default_acceleration = 0U;
};

class ZdtMotor final : public motor::IVelocityControl, public motor::IPositionControl {
   public:
    ZdtMotor(UartBus& uart, uint8_t address, const ZdtMotorConfig& config = {});

    motor::Status SetTargetVelocityRadPerSec(float velocity_rad_per_sec) override;
    motor::Status SendVelocity() override;
    motor::Status SetTargetPositionRad(float position_rad) override;

    motor::Status Send() { return velocity_mode_ ? SendVelocity() : SendPosition(); }
    motor::Status SendPosition() override;

    motor::Status Enable(bool enabled, bool clear_errors = false);
    motor::Status Stop();
    motor::Status SetZero();
    motor::Status QueryPosition();
    motor::Status QueryVelocity();
    motor::Status QueryStatus();

    float PositionRad() const { return position_rad_; }
    float VelocityRadPerSec() const { return velocity_rad_per_sec_; }
    uint8_t StatusFlags() const { return status_flags_; }
    uint8_t ErrorCode() const { return error_code_; }
    bool ResponsePending() const { return response_pending_; }
    void BeginPending(uint8_t function);
    void ResetPending();
    void Receive(const uint8_t* data, size_t size);

   private:
    static void OnReceive(void* context, const uint8_t* data, size_t size);
    static void OnError(void* context, uint32_t error_code);
    void Error(uint32_t error_code);
    motor::Status SendFrame(const uint8_t* data, size_t size, uint8_t function);
    motor::Status SendVelocityFrame(float velocity_rad_per_sec);
    motor::Status SendPositionFrame(float position_rad);

    UartBus& uart_;
    uint8_t address_;
    ZdtMotorConfig config_;
    float target_velocity_rad_per_sec_ = 0.0f;
    float target_position_rad_ = 0.0f;
    float position_rad_ = 0.0f;
    float velocity_rad_per_sec_ = 0.0f;
    uint8_t status_flags_ = 0U;
    uint8_t error_code_ = 0U;
    uint8_t expected_function_ = 0U;
    bool velocity_mode_ = false;
    bool response_pending_ = false;
    uint8_t response_[16] = {};
    size_t response_size_ = 0U;
};

#endif  // THREE_FRICTION_DRV_ZDT_MOTOR_HPP
