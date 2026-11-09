#ifndef THREE_FRICTION_MOTOR_BASE_HPP
#define THREE_FRICTION_MOTOR_BASE_HPP

#include <cstdint>

// 控制能力接口：彼此独立，互不继承。
// 上层只依赖自己真正需要的能力，驱动只承诺自己能做的事。
namespace motor
{
        enum class Status : uint8_t
        {
                kOk,
                kBusy,
                kUnsupported,
                kInvalidArgument,
                kNotReady,
                kIoError,
        };

        class ITorqueControl
        {
                public:
                        virtual ~ITorqueControl() = default;

                        // 保存并钳位目标扭矩（N·m）；只表示已接受，不代表已到位
                        virtual Status SetTargetTorqueNm(float torque_nm) = 0;

                        // 提交已保存的目标扭矩
                        virtual Status SendTorque() = 0;
        };

        class IVelocityControl
        {
                public:
                        virtual ~IVelocityControl() = default;

                        // 保存目标速度（RPM）；只表示已接受
                        virtual Status SetTargetVelocityRpm(float velocity_rpm) = 0;

                        // 提交已保存的目标速度
                        virtual Status SendVelocity() = 0;
        };

        class IPositionControl
        {
                public:
                        virtual ~IPositionControl() = default;

                        // 保存目标位置（度）；只表示已接受，不代表已到位
                        virtual Status SetTargetPositionDeg(float position_deg) = 0;

                        // 提交已保存的目标位置
                        virtual Status SendPosition() = 0;
        };

        // 使能能力：仅驱动硬件真的支持软件使能时继承
        class IEnableControl
        {
                public:
                        virtual ~IEnableControl() = default;

                        // 使能电机
                        virtual Status Enable() = 0;

                        // 失能电机（失去保持力矩）
                        virtual Status Disable() = 0;
        };

        // MIT 五参数命令（位置/速度/kp/kd/前馈力矩），全部度制
        struct MitCommand
        {
                float position_deg         = 0.0f;
                float velocity_rpm = 0.0f;
                float kp                   = 0.0f; // N·m/deg
                float kd                   = 0.0f; // N·m·s/deg
                float feedforward_torque_nm = 0.0f;
        };

        // MIT 能力：位置/速度/kp/kd/前馈力矩一次下发
        class IMitControl
        {
                public:
                        virtual ~IMitControl() = default;

                        // 按 MIT 五参数下发命令
                        virtual Status SendMit(const MitCommand &command) = 0;
        };
} // namespace motor

#endif  // THREE_FRICTION_MOTOR_BASE_HPP
