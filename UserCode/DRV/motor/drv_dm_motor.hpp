#ifndef THREE_FRICTION_DRV_DM_MOTOR_HPP
#define THREE_FRICTION_DRV_DM_MOTOR_HPP

#include <array>
#include <cstdint>

#include "hal_can.hpp"
#include "motor_base.hpp"

struct DmMotorSpec {
    // 以下量程与设备 PMAX/VMAX/TMAX 对应，位置/速度为度，扭矩为 N·m
    float position_min_deg = -180.0f;
    float position_max_deg = 180.0f;
    float velocity_min_deg_per_sec = -1718.8734f;  // ≈ -30 rad/s
    float velocity_max_deg_per_sec = 1718.8734f;   // ≈ +30 rad/s
    float torque_min_nm = -12.0f;
    float torque_max_nm = 12.0f;
    float kp_max = 500.0f;
    float kd_max = 5.0f;
};

// 兼容旧名：MIT 命令结构已提升到 motor 命名空间
using DmMitCommand = motor::MitCommand;

class DmMotor final : public motor::ITorqueControl,
                      public motor::IEnableControl,
                      public motor::IMitControl,
                      public CanNode {
   public:
    DmMotor(CAN_HandleTypeDef* hcan, uint32_t command_id, uint32_t feedback_id,
            const DmMotorSpec& spec = {});

    // 解析反馈帧：错误码、位置/速度/扭矩与温度
    HAL_StatusTypeDef OnFrame(const uint8_t* data, uint16_t length) override;
    // 保存并钳位目标扭矩（MIT 模式下以 kp=kd=0 纯扭矩输出）
    motor::Status SetTargetTorqueNm(float torque_nm) override;
    // 提交已保存的目标扭矩
    motor::Status SendTorque() override;
    // 按 MIT 五参数下发（位置/速度/kp/kd/前馈力矩），kp/kd 入参为度制
    motor::Status SendMit(const motor::MitCommand& command) override;
    // 发送使能帧
    motor::Status Enable() override;
    // 发送失能帧
    motor::Status Disable() override;
    // 清除驱动器错误
    HAL_StatusTypeDef ClearError();
    // 将当前位置设为零点；permanent=true 时写入驱动器存储
    HAL_StatusTypeDef SetZero(bool permanent);
    // 位置速度模式：帧 ID 为 command_id+0x100
    motor::Status SendPositionVelocity(float position_deg, float max_velocity_deg_per_sec,
                                       float kd);
    // 速度模式：帧 ID 为 command_id+0x200
    motor::Status SendVelocity(float velocity_deg_per_sec, float torque_limit_nm);

    // 反馈：位置/速度/扭矩/温度/错误码
    float PositionDeg() const { return position_deg_; }
    float VelocityDegPerSec() const { return velocity_deg_per_sec_; }
    float TorqueNm() const { return torque_nm_; }
    int8_t TemperatureC() const { return temperature_c_; }
    uint8_t ErrorCode() const { return error_code_; }

   private:
    // 浮点→定点线性映射（按位宽上位）
    static uint16_t FloatToUInt(float value, float min, float max, uint8_t bits);
    // 浮点按小端写入 4 字节
    static void PutFloatLe(uint8_t* data, float value);
    // 统一出口：校验长度后发标准帧
    HAL_StatusTypeDef SendCommand(const uint8_t* data, uint16_t length);

    DmMotorSpec spec_;
    float target_torque_nm_ = 0.0f;
    float position_deg_ = 0.0f;
    float velocity_deg_per_sec_ = 0.0f;
    float torque_nm_ = 0.0f;
    int8_t temperature_c_ = 0;
    uint8_t error_code_ = 0U;
};

#endif  // THREE_FRICTION_DRV_DM_MOTOR_HPP
