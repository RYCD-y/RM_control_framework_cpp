//
// Created by ye on 2026/10/30.
//

#include "debug.hpp"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "drv_dji_motor.hpp"
#include "hal_can.hpp"
#include "cotr.hpp"
#include "vofa.hpp"

// void debugTask(void *ptr)
// {
//         (void) ptr;
//
//         static constexpr TickType_t kMotorCtrlPeriodMs = 1U;
//
//         static DjiMotor m1(&hcan1, 0x1FF, 0x205, kGM6020_Voltage);
//         DjiMotor::groupInit(&m1);
//
//         static Pid pid_speed = {};
//
//         pid_speed.init(
//                        m1.getMaxTorque(), m1.getMaxTorque() * 0.5f,
//                        3,
//                        0.005f, 0.002f, 0,
//                        0, 0,
//                        0.02, 0,
//                        0,
//                        Pid::IntegralLimit | Pid::OutputFilter
//                       );
//
//                 float target_speed = 2000.0f;
//
//         TickType_t last_wake = xTaskGetTickCount();
//         for (;;)
//         {
//
//                 m1.setTorque(pid_speed.calculate(m1.getSpeed(), target_speed));
//
//                 m1.groupSendTorque();
//
//                 vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kMotorCtrlPeriodMs));
//         }
// }

void debugTask(void *ptr)
{
        (void) ptr;

        static constexpr TickType_t kMotorCtrlPeriodMs = 1U;

        // 过滤器 + HAL_CAN_Start 已在 ent_main() 里做完（必须在任务启动前）

        // // 三台 C620/M3508：共用控制帧 0x200，反馈 ID 0x201~0x203
        // static djiMotor m1(&hcan2, 0x200, 0x201, kC620_M3508);
        // static djiMotor m2(&hcan2, 0x200, 0x202, kC620_M3508);
        // static djiMotor m3(&hcan2, 0x200, 0x203, kC620_M3508);
        //
        // // GM6020 电压控制：控制帧 0x1FF，反馈 ID 0x205~0x208
        // static djiMotor g1(&hcan2, 0x1FF, 0x205, kGM6020_Voltage);
        //
        // djiMotor::groupInit(&m1, &m2, &m3);   // 3 台即可，第 4 槽自动为空
        // djiMotor::groupInit(&g1);             // 单独一组（0x1FF）

        static DjiMotor m1(&hcan1, 0x1FF, 0x205, kGM6020_Voltage);
        DjiMotor::groupInit(&m1);

        float speed_max = 2000.0f;

        static Pid pid_speed = {};
        static Pid pid_angle = {};


        pid_speed.init(
                       m1.getMaxTorque(), m1.getMaxTorque() * 0.5f,
                       3,
                       0.005f, 0.002f, 0,
                       0, 0,
                       0.02f, 0,
                       0,
                       Pid::IntegralLimit | Pid::OutputFilter | Pid::ErrorHandle
                      );

        pid_angle.init(
                       speed_max, speed_max * 0.5f,
                       0,
                       10.0f, 0, 0.2,
                       0, 0,
                       0.02f, 0,
                       0,
                       Pid::IntegralLimit | Pid::OutputFilter
                      );

        static float target_angle  = 0.0f;
        static float target_speed  = 0.0f;
        static float target_torque = 0.0f;
        static int   i = 0, j = 0;


        TickType_t last_wake = xTaskGetTickCount();
        for (;;)
        {
                // m1.sendTorque(0.0f);
                // m2.sendTorque(0.0f);
                // m3.sendTorque(0.0f);
                // g1.sendTorque(0.0f);
                //
                // m1.groupSendTorque();      // 一帧 0x200 带 3 台
                // g1.groupSendTorque();      // 一帧 0x1FF 带 1 台

                if (i++ % 1000 == 0) j++;

                target_angle  = 180.0f * static_cast<float>(j);
                target_speed  = pid_angle.calculate(m1.getTotalAngle(), target_angle);
                target_torque = pid_speed.calculate(m1.getTotalAngleSpeed(), target_speed);

                m1.setTorque(target_torque);

                m1.groupSendTorque();

                vofa::set(0, target_angle);
                vofa::set(1, m1.getTotalAngle());
                vofa::set(2, target_torque);
                vofa::set(3, m1.getTorque());
                vofa::set(4, pid_speed.blocked());
                vofa::set(5, pid_speed.errorCount());


                vofa::flush();

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kMotorCtrlPeriodMs));
        }
}
