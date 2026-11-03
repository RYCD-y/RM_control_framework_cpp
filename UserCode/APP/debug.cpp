//
// Created by ye on 2026/10/30.
//

#include "debug.hpp"

#include <cmath>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "drv_dji_motor.hpp"
#include "ent_devices.hpp"
#include "hal_can.hpp"
#include "cotr.hpp"
#include "vofa.hpp"


void debugTask(void *ptr)
{
        (void) ptr;

        static constexpr TickType_t kMotorCtrlPeriodMs = 1U;

        // 拨弹盘：M2006 内部 36:1 × 外部同步带 55/17 → 总减速比 1980/17 ≈ 116.47
        // （数值取自 QX_CK/Project/Robot_Application/Define.h 的 Triger_Reduction）
        constexpr djiMotorSpec kC610_M2006_Trigger =
                        withExtraGear(kC610_M2006, 55.0f / 17.0f, "M2006/trigger");


        float           speed_max = 10000.0f;
        //电机初始化
        static DjiMotor friction[3] = {
                DjiMotor(&hcan1, 0x200, 0x201, kC610_M2006_Direct),
                DjiMotor(&hcan1, 0x200, 0x202, kC610_M2006_Direct),
                DjiMotor(&hcan1, 0x200, 0x203, kC610_M2006_Direct)
        };

        // static DjiMotor friction1(&hcan1, 0x200, 0x201, kC610_M2006_Direct);
        // static DjiMotor friction2(&hcan1, 0x200, 0x202, kC610_M2006_Direct);
        // static DjiMotor friction3(&hcan1, 0x200, 0x203, kC610_M2006_Direct);
        static DjiMotor trigger(&hcan1, 0x200, 0x204, kC610_M2006_Trigger);

        DjiMotor::groupInit(
                            &friction[0],
                            &friction[1],
                            &friction[2],
                            &trigger
                           );

        //PID测试
        static Pid trigger_pid_speed = {};
        static Pid trigger_pid_angle = {};

        trigger_pid_speed.init(
                               trigger.getMaxTorque(), trigger.getMaxTorque() * 0.5f,
                               3,
                               0.04f, 0.005f, 0,
                               0, 0,
                               0.0f, 0,
                               0,
                               Pid::IntegralLimit | Pid::OutputFilter | Pid::ErrorHandle
                              );

        trigger_pid_angle.init(
                               speed_max, speed_max * 0.5f,
                               0,
                               7.0f, 0, 0.6,
                               0, 0,
                               0.02f, 0,
                               0,
                               Pid::IntegralLimit | Pid::OutputFilter
                              );

        static Pid friction_pid_speed[3];
        for (Pid &pid : friction_pid_speed)
        {
                pid.init(
                         friction[0].getMaxTorque(), friction[0].getMaxTorque() * 0.5f,
                         5,
                         0.00005f, 0.0f, 0.0f,
                         0, 0,
                         0.0f, 0,
                         0,
                         Pid::IntegralLimit | Pid::OutputFilter
                        );
        }

        static float trigger_target_angle  = 0.0f;
        static float trigger_target_speed  = 0.0f;
        static float trigger_target_torque = 0.0f;

        static float friction_target_speed[3]  = {};
        static float friction_target_torque[3] = {};

        static int i = 0, j = 0;

        // ── 堵转退弹（最小测试版）──
        // 拨弹盘 9 发/圈 → 每发 40°；getTotalAngle() 已是拨弹盘角度（减速比已含 55/17）
        static constexpr float kBulletDeg = 360.0f / 9.0f;
        static bool            reversing  = false;


        vTaskDelay(3000);
        TickType_t last_wake = xTaskGetTickCount();
        for (;;)
        {
                if (i++ % 500 == 0) j++;

                // 堵转：回退 40° 并复位（reset 才能清掉 MotorBlocked）
                if (trigger_pid_speed.blocked())
                {
                        reversing            = true;
                        trigger_target_angle = trigger.getTotalAngle() - kBulletDeg; // 退 1 发
                        trigger_pid_angle.reset();
                        trigger_pid_speed.reset();
                }

                if (!reversing)
                {
                        trigger_target_angle = kBulletDeg * static_cast<float>(j);
                }
                else if (std::fabs(trigger.getTotalAngle() - trigger_target_angle) < 5.0f)
                {
                        reversing = false; // 退弹到位，恢复追踪
                }

                trigger_target_speed  = trigger_pid_angle.calculate(trigger.getTotalAngle(), trigger_target_angle);
                trigger_target_torque = trigger_pid_speed.calculate(trigger.getTotalAngleSpeed(), trigger_target_speed);
                //设置拨弹盘力矩
                trigger.setTorque(trigger_target_torque);

                for (int k = 0; k < 3; k++)
                {
                        friction_target_speed[k] = 9000.0f;
                        if (k == 0 || k == 1) friction_target_speed[k] = -friction_target_speed[k];

                        friction_target_torque[k] = friction_pid_speed[k].calculate(friction[k].getTotalAngleSpeed(), friction_target_speed[k]);
                        friction[k].setTorque(friction_target_torque[k]);
                }


                trigger.groupSendTorque();

                // vofa::set(0, trigger_target_speed);
                // vofa::set(1, trigger.getTotalAngleSpeed());
                // vofa::set(2, trigger_target_torque);
                // vofa::set(3, trigger.getTorque());
                // vofa::set(4, trigger_pid_speed.blocked());
                // vofa::set(5, trigger_pid_speed.errorCount());

                vofa::set(0, friction_target_speed[0]);
                vofa::set(1, friction[0].getTotalAngleSpeed());
                vofa::set(2, friction_target_torque[0]);
                vofa::set(3, friction[0].getTorque());
                // vofa::set(4, trigger_pid_speed.blocked());
                // vofa::set(5, trigger_pid_speed.errorCount());


                vofa::flush();

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kMotorCtrlPeriodMs));
        }
}
