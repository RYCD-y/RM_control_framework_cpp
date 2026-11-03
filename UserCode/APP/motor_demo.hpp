#ifndef THREE_FRICTION_MOTOR_DEMO_HPP
#define THREE_FRICTION_MOTOR_DEMO_HPP

#include <cstddef>

#include "motor_base.hpp"

// APP 层小示例：上层只依赖"能力接口"，不关心具体电机型号。
//
// 用法（由调用方显式构造电机并决定是否使能，示例故意不自动启动硬件）：
//
//   ZdtMotor zdt(uart1, 1);                 // 位置/速度能力
//   PwmServo servo(&htim1, TIM_CHANNEL_1, kReferenceServoCalibration);
//   servo.Init();
//   servo.Enable();
//
//   motor::IPositionControl* arms[] = {&zdt, &servo};
//   motor_demo::DrivePositions(arms, 2, 0.5f);   // 同一段代码驱动两种电机
//
//   DjiMotor wheel(&hcan1, 0x200, 0x201, kC610_M2006_Direct);
//   DmMotor  joint(&hcan1, 0x01, 0x11);
//   motor::ITorqueControl* wheels[] = {&wheel, &joint};
//   motor_demo::DriveTorques(wheels, 2, 1.0f);
namespace motor_demo {

// 给一组"支持位置控制"的电机下发同一目标位置并提交。
// 返回最后一个非 kOk 的状态；全部成功返回 kOk。
motor::Status DrivePositions(motor::IPositionControl* const* motors, size_t count,
                             float position_rad);

// 给一组"支持扭矩控制"的电机下发同一目标扭矩并提交。
motor::Status DriveTorques(motor::ITorqueControl* const* motors, size_t count, float torque_nm);

// 给一组"支持速度控制"的电机下发同一目标速度并提交。
motor::Status DriveVelocities(motor::IVelocityControl* const* motors, size_t count,
                              float velocity_rad_per_sec);

}  // namespace motor_demo

#endif  // THREE_FRICTION_MOTOR_DEMO_HPP
