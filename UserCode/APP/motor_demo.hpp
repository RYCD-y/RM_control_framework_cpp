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
//   motor_demo::DrivePositions(arms, 2, 90.0f);   // 90 度；同一段代码驱动两种电机
//
//   DjiMotor wheel(&hcan1, 0x200, 0x201, kC610_M2006_Direct);
//   DmMotor  joint(&hcan1, 0x01, 0x11, kDmJ4340);   // 显式选预设，勿依赖默认值
//   motor::ITorqueControl* wheels[] = {&wheel, &joint};
//   motor_demo::DriveTorques(wheels, 2, 1.0f);
//
//   motor::IVelocityControl* spinners[] = {&zdt};
//   motor_demo::DriveVelocities(spinners, 1, 360.0f);   // 360 RPM
//
//   // 使能：只有真正有软件使能命令的电机才能放进这个数组
//   motor::IEnableControl* enablables[] = {&zdt, &servo, &joint};
//   motor_demo::SetEnabled(enablables, 3, true);
//
//   // MIT：位置/速度/kp/kd 全部度制，前馈力矩 N·m
//   motor::IMitControl* mits[] = {&joint};
//   motor_demo::SendMitToAll(mits, 1, {90.0f, 0.0f, 2.0f, 0.02f, 0.0f});
//
//   // VESC：与上面三个能力接口完全同构，可直接参与批量驱动
//   VescMotor vesc(&hcan2, 1, kVescM3508);      // controller_id = VESC ID
//   motor::ITorqueControl* with_vesc[] = {&wheel, &joint, &vesc};
//   motor_demo::DriveTorques(with_vesc, 3, 1.0f);
//   (void)vesc.SetDuty(0.2f);                    // VESC 专有：占空比
//   (void)vesc.SetBrakeCurrent(3.0f);            // VESC 专有：制动电流
//   float output_rpm = vesc.VelocityRpm(); // STATUS_1 反馈（输出轴 RPM）
namespace motor_demo {

// 给一组"支持位置控制"的电机下发同一目标位置并提交。
// 返回最后一个非 kOk 的状态；全部成功返回 kOk。
motor::Status DrivePositions(motor::IPositionControl* const* motors, size_t count,
                             float position_deg);

// 给一组"支持扭矩控制"的电机下发同一目标扭矩并提交。
motor::Status DriveTorques(motor::ITorqueControl* const* motors, size_t count, float torque_nm);

// 给一组"支持速度控制"的电机下发同一目标速度并提交。
motor::Status DriveVelocities(motor::IVelocityControl* const* motors, size_t count,
                              float velocity_rpm);

// 统一使能/失能一组电机；不支持使能的类型不会被误传进来（编译期拦截）。
motor::Status SetEnabled(motor::IEnableControl* const* motors, size_t count, bool enabled);

// 给一组支持 MIT 的电机下发同一 MIT 命令。
motor::Status SendMitToAll(motor::IMitControl* const* motors, size_t count,
                           const motor::MitCommand& command);

}  // namespace motor_demo

#endif  // THREE_FRICTION_MOTOR_DEMO_HPP
