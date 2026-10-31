//
// Created by ye on 2026/10/30.
//

#include "debug.hpp"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "drv_dji_motor.hpp"
#include "hal_can.hpp"
#include "pid.hpp"
#include "vofa.hpp"

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

        static djiMotor m1(&hcan1, 0x1FF, 0x205, kGM6020_Voltage);
        djiMotor::groupInit(&m1);

        static Pid pid_speed = {};

        pid_speed.init(
                       m1.getMaxTorque(), m1.getMaxTorque() * 0.5f,
                       3,
                       0.005f, 0.002f, 0,
                       0, 0,
                       0.02, 0,
                       0,
                       Pid::IntegralLimit | Pid::OutputFilter
                      );

                float target_speed = 2000.0f;

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

                m1.setTorque(pid_speed.calculate(m1.getSpeed(), target_speed));

                m1.groupSendTorque();

                vofa::set(0, target_speed);
                vofa::set(1, m1.getSpeed());
                vofa::set(2, m1.getTargetTorque());
                vofa::set(3, m1.getTorque());

                vofa::flush();

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kMotorCtrlPeriodMs));
        }
}
