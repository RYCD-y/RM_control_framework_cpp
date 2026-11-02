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


void debugTask(void *ptr)
{
        (void) ptr;

        static constexpr TickType_t kMotorCtrlPeriodMs = 1U;

        float           speed_max = 10000.0f;
        //电机初始化
        static DjiMotor friction1(&hcan1, 0x200, 0x201, kC610_M2006_Direct);
        static DjiMotor friction2(&hcan1, 0x200, 0x202, kC610_M2006_Direct);
        static DjiMotor friction3(&hcan1, 0x200, 0x203, kC610_M2006_Direct);
        static DjiMotor trigger(&hcan1, 0x200, 0x204, kC610_M2006);

        DjiMotor::groupInit(
                            &friction1,
                            &friction2,
                            &friction3,
                            &trigger
                           );

        //PID测试
        static Pid trigger_pid_speed = {};
        static Pid trigger_pid_angle = {};

        trigger_pid_speed.init(
                               trigger.getMaxTorque(), trigger.getMaxTorque() * 0.5f,
                               3,
                               0.005f, 0.002f, 0,
                               0, 0,
                               0.02f, 0,
                               0,
                               Pid::IntegralLimit | Pid::OutputFilter | Pid::ErrorHandle
                              );

        trigger_pid_angle.init(
                               speed_max, speed_max * 0.5f,
                               0,
                               10.0f, 0, 0.2,
                               0, 0,
                               0.02f, 0,
                               0,
                               Pid::IntegralLimit | Pid::OutputFilter
                              );

        static Pid friction_pid_speed[3];
        for (Pid &pid : friction_pid_speed)
        {
                pid.init(
                         trigger.getMaxTorque(), trigger.getMaxTorque() * 0.5f,
                         3,
                         0.005f, 0.002f, 0.0f,
                         0, 0,
                         0.02f, 0,
                         0,
                         Pid::IntegralLimit | Pid::OutputFilter
                        );
        }

        static float target_angle  = 0.0f;
        static float target_speed  = 0.0f;
        static float target_torque = 0.0f;
        static int   i             = 0, j = 0;


        TickType_t last_wake = xTaskGetTickCount();
        for (;;)
        {
                if (i++ % 1000 == 0) j++;

                target_angle  = 40.0f * static_cast<float>(j);
                target_speed  = trigger_pid_angle.calculate(trigger.getTotalAngle(), target_angle);
                target_torque = trigger_pid_speed.calculate(trigger.getTotalAngleSpeed(), target_speed);

                trigger.setTorque(target_torque);

                trigger.groupSendTorque();

                vofa::set(0, target_angle);
                vofa::set(1, trigger.getTotalAngle());
                vofa::set(2, target_torque);
                vofa::set(3, trigger.getTorque());
                vofa::set(4, trigger_pid_speed.blocked());
                vofa::set(5, trigger_pid_speed.errorCount());


                vofa::flush();

                if (trigger_pid_speed.blocked() == Pid::MotorBlocked)
                {

                }

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kMotorCtrlPeriodMs));
        }
}
