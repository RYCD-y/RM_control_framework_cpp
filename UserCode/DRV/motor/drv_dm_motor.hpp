#ifndef THREE_FRICTION_DRV_DM_MOTOR_HPP
#define THREE_FRICTION_DRV_DM_MOTOR_HPP

#include <array>
#include <cstdint>

#include "hal_can.hpp"
#include "motor_base.hpp"

// 达妙控制模式编码（对应驱动器寄存器 0x0A 的值）
// 5~7（CSP/CSV/CST）仅部分型号支持，本期不建模
enum class DmCtrlMode : uint8_t
{
        kMit              = 1,
        kPositionVelocity = 2,
        kVelocity         = 3,
        kForcePosition    = 4,
};

struct DmMotorSpec
{
        // 驱动器当前被配置成的控制模式；不匹配的命令会被拒绝
        DmCtrlMode mode = DmCtrlMode::kMit;

        // 以下量程与设备 PMAX/VMAX/TMAX 对应：位置为度，速度为 RPM，扭矩为 N·m
        float position_min_deg         = -180.0f;
        float position_max_deg         = 180.0f;
        float velocity_min_rpm = -286.4789f;  // ≈ -30 rad/s
        float velocity_max_rpm = 286.4789f;   // ≈ +30 rad/s
        float torque_min_nm            = -12.0f;
        float torque_max_nm            = 12.0f;
        float kp_max                   = 500.0f;
        float kd_max                   = 5.0f;
};

// ─────────────── 预设（必须与驱动器实际 PMAX/VMAX/TMAX 一致）───────────────
//
// 以下量程取自达妙手册的 MIT 演示预设，**不是型号额定值**，而是映射范围：
//   J4340 手册示例：PMAX ±12.5 rad、VMAX ±30 rad/s、TMAX ±10 N·m
//   JH11  手册示例：PMAX ±12.5 rad、VMAX ±10 rad/s、TMAX ±12 N·m
// 换算到度/RPM：12.5 rad = 716.1972°，30 rad/s = 286.4789 rpm，10 rad/s = 95.4930 rpm。
// 上机前必须用调试助手或“读取参数”命令核对 0x15/0x16/0x17 的实值。

// J4340：40:1，额定 9 N·m / 36 rpm
inline constexpr DmMotorSpec kDmJ4340{
        DmCtrlMode::kMit,
        -716.1972f, 716.1972f,        // PMAX ±12.5 rad
        -286.4789f, 286.4789f,        // VMAX ±30 rad/s
        -10.0f, 10.0f,                // TMAX ±10 N·m
        500.0f, 5.0f};

// JH11（51:1 与 101:1 共用同一套映射范围，减速比在驱动器内部处理）
inline constexpr DmMotorSpec kDmJH11{
        DmCtrlMode::kMit,
        -716.1972f, 716.1972f,        // PMAX ±12.5 rad
        -95.4930f, 95.4930f,          // VMAX ±10 rad/s
        -12.0f, 12.0f,                // TMAX ±12 N·m
        500.0f, 5.0f};

// J4310：手册未给 MIT 预设，沿用 DM 系列演示值，必须按实物校准
// （10:1，额定 3.5 N·m / 120 rpm）
inline constexpr DmMotorSpec kDmJ4310{
        DmCtrlMode::kMit,
        -716.1972f, 716.1972f,
        -286.4789f, 286.4789f,
        -10.0f, 10.0f,
        500.0f, 5.0f};

// 默认值：与 kDmJ4310 相同，仅作为“未配置”占位，实际使用务必显式传入预设
inline constexpr DmMotorSpec kDmDefault = kDmJ4310;

// 兼容旧名：MIT 命令结构已提升到 motor 命名空间
using DmMitCommand = motor::MitCommand;

class DmMotor final : public motor::ITorqueControl,
                      public motor::IEnableControl,
                      public motor::IMitControl,
                      public CanNode {
   public:
    DmMotor(CAN_HandleTypeDef* hcan, uint32_t command_id, uint32_t feedback_id,
            const DmMotorSpec& spec = kDmDefault);

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
    motor::Status SendPositionVelocity(float position_deg, float max_velocity_rpm,
                                       float kd);
    // 速度模式：帧 ID 为 command_id+0x200
    motor::Status SendVelocity(float velocity_rpm, float torque_limit_nm);

    // 反馈：位置/速度/扭矩/温度/错误码
    float PositionDeg() const { return position_deg_; }
    float VelocityRpm() const { return velocity_rpm_; }
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
    float velocity_rpm_ = 0.0f;
    float torque_nm_ = 0.0f;
    int8_t temperature_c_ = 0;
    uint8_t error_code_ = 0U;
};

#endif  // THREE_FRICTION_DRV_DM_MOTOR_HPP
