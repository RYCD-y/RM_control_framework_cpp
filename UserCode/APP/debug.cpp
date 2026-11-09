#include "debug.hpp"

#include <cmath>

#include "FreeRTOS.h"
#include "cotr.hpp"
#include "drv_dji_motor.hpp"
#include "drv_vesc_motor.hpp"
#include "hal_can.hpp"
#include "task.h"
#include "vofa.hpp"

// 沿用参考工程控制流程，第三台摩擦轮由 VESC 内部完成速度闭环。
void DebugTask(void* ptr) {
    (void)ptr;

    static constexpr TickType_t kMotorCtrlPeriodMs = 1U;

    // 拨弹盘：M2006 内部 36:1 × 外部同步带 55/17。
    constexpr djiMotorSpec kC610M2006Trigger =
        WithExtraGear(kC610_M2006, 55.0f / 17.0f, "M2006/trigger");

    float speed_max = 10000.0f;
    static DjiMotor friction[2] = {
        DjiMotor(&hcan1, 0x200, 0x201, kC610_M2006_Direct),
        DjiMotor(&hcan1, 0x200, 0x202, kC610_M2006_Direct)
    };

    // 与原摩擦轮直驱规格一致：目标 RPM 是转子 RPM，而非 36:1 输出轴 RPM。
    constexpr VescSpec kVescM2006Friction{
        kVescM2006.pole_pairs, 1.0f,
        kVescM2006.kt_output_nm_per_a / kVescM2006.gear_ratio,
        kVescM2006.current_max_a, kVescM2006.erpm_max
    };
    static VescMotor friction_vesc(&hcan1, 30, kVescM2006Friction);
    static DjiMotor trigger(&hcan1, 0x200, 0x204, kC610M2006Trigger);

    // 0x203 留空，0x204 的拨弹盘仍在原槽位。
    (void)DjiMotor::GroupInit(&friction[0], &friction[1], nullptr, &trigger);

    static Pid trigger_pid_speed = {};
    static Pid trigger_pid_angle = {};

    trigger_pid_speed.Init(
        trigger.MaxTorqueNm(), trigger.MaxTorqueNm() * 0.5f,
        3, 0.04f, 0.005f, 0,
        0, 0, 0.0f, 0, 0,
        Pid::kIntegralLimit | Pid::kOutputFilter | Pid::kErrorHandle);

    trigger_pid_angle.Init(
        speed_max, speed_max * 0.5f,
        0, 7.0f, 0, 0.6f,
        0, 0, 0.02f, 0, 0,
        Pid::kIntegralLimit | Pid::kOutputFilter);

    // 主控只计算两台 DJI 摩擦轮的速度 PID。
    static Pid friction_pid_speed[2];
    for (Pid& pid : friction_pid_speed) {
        pid.Init(
            friction[0].MaxTorqueNm(), friction[0].MaxTorqueNm() * 0.5f,
            5, 0.00005f, 0.0f, 0.0f,
            0, 0, 0.0f, 0, 0,
            Pid::kIntegralLimit | Pid::kOutputFilter);
    }

    static float trigger_target_angle = 0.0f;
    static float trigger_target_speed = 0.0f;
    static float trigger_target_torque = 0.0f;

    static float friction_target_speed[3] = {};
    static float friction_target_torque[2] = {};

    static int i = 0, j = 0;

    // 拨弹盘 9 发/圈，每发 40 度；累计角已包含外部减速比。
    static constexpr float kBulletDeg = 360.0f / 9.0f;
    static bool reversing = false;

    vTaskDelay(3000);
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        // 堵转时退回一发并复位 PID。
        if (trigger_pid_speed.Blocked()) {
            reversing = true;
            trigger_target_angle = trigger.TotalAngleDeg() - kBulletDeg;
            trigger_pid_angle.Reset();
            trigger_pid_speed.Reset();
        }

        if (!reversing) {
            if (i++ % 200 == 0) j++;
            trigger_target_angle = kBulletDeg * static_cast<float>(j);
        } else if (std::fabs(trigger.TotalAngleDeg() - trigger_target_angle) < 5.0f) {
            reversing = false;
        }

        trigger_target_speed =
            trigger_pid_angle.Calculate(trigger.TotalAngleDeg(), trigger_target_angle);
        trigger_target_torque =
            trigger_pid_speed.Calculate(trigger.TotalAngleSpeedRpm(), trigger_target_speed);
        (void)trigger.SetTargetTorqueNm(trigger_target_torque);

        for (int k = 0; k < 3; k++) {
            friction_target_speed[k] = 10000.0f;
            if (k == 0 || k == 1 || k == 2) friction_target_speed[k] = -friction_target_speed[k];

            if (k < 2) {
                friction_target_torque[k] = friction_pid_speed[k].Calculate(
                    friction[k].TotalAngleSpeedRpm(), friction_target_speed[k]);
                (void)friction[k].SetTargetTorqueNm(friction_target_torque[k]);
            } else {
                // VESC 直接接收 RPM，主控不计算其 PID。
                (void)friction_vesc.SetTargetVelocityRpm(friction_target_speed[k]);
            }
        }

        (void)trigger.GroupSendTorque();
        (void)friction_vesc.SendVelocity();

        // 保留参考工程的 VOFA 通道布局。
        vofa::Set(0, friction_target_speed[0]);
        vofa::Set(1, friction_vesc.VelocityRpm());
        vofa::Set(2, friction[0].TotalAngleSpeedRpm());
        vofa::Set(3, friction[1].TotalAngleSpeedRpm());
        (void)vofa::Flush();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kMotorCtrlPeriodMs));
    }
}
