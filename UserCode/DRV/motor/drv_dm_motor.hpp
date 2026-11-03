#ifndef THREE_FRICTION_DRV_DM_MOTOR_HPP
#define THREE_FRICTION_DRV_DM_MOTOR_HPP

#include <array>
#include <cstdint>

#include "hal_can.hpp"
#include "motor_base.hpp"

struct DmMotorSpec {
    float position_min_rad = -3.1415926f;
    float position_max_rad = 3.1415926f;
    float velocity_min_rad_per_sec = -30.0f;
    float velocity_max_rad_per_sec = 30.0f;
    float torque_min_nm = -12.0f;
    float torque_max_nm = 12.0f;
    float kp_max = 500.0f;
    float kd_max = 5.0f;
};

struct DmMitCommand {
    float position_rad = 0.0f;
    float velocity_rad_per_sec = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float feedforward_torque_nm = 0.0f;
};

class DmMotor final : public motor::ITorqueControl, public CanNode {
   public:
    DmMotor(CAN_HandleTypeDef* hcan, uint32_t command_id, uint32_t feedback_id,
            const DmMotorSpec& spec = {});

    HAL_StatusTypeDef OnFrame(const uint8_t* data, uint16_t length) override;
    motor::Status SetTargetTorqueNm(float torque_nm) override;
    motor::Status SendTorque() override;
    motor::Status SendMit(const DmMitCommand& command);
    HAL_StatusTypeDef Enable();
    HAL_StatusTypeDef Disable();
    HAL_StatusTypeDef ClearError();
    HAL_StatusTypeDef SetZero(bool permanent);
    motor::Status SendPositionVelocity(float position_rad, float max_velocity_rad_per_sec,
                                       float kd);
    motor::Status SendVelocity(float velocity_rad_per_sec, float torque_limit_nm);

    float PositionRad() const { return position_rad_; }
    float VelocityRadPerSec() const { return velocity_rad_per_sec_; }
    float TorqueNm() const { return torque_nm_; }
    int8_t TemperatureC() const { return temperature_c_; }
    uint8_t ErrorCode() const { return error_code_; }

   private:
    static uint16_t FloatToUInt(float value, float min, float max, uint8_t bits);
    static void PutFloatLe(uint8_t* data, float value);
    HAL_StatusTypeDef SendCommand(const uint8_t* data, uint16_t length);

    DmMotorSpec spec_;
    float target_torque_nm_ = 0.0f;
    float position_rad_ = 0.0f;
    float velocity_rad_per_sec_ = 0.0f;
    float torque_nm_ = 0.0f;
    int8_t temperature_c_ = 0;
    uint8_t error_code_ = 0U;
};

#endif  // THREE_FRICTION_DRV_DM_MOTOR_HPP
