//
// Created by ye on 2026/10/30.
//

#include "debug.hpp"

#include "drv_dji_motor.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "hal_can.hpp"

void debugTask(void *ptr)
{
        static constexpr TickType_t kMotorCtrlPeriodMs = 1U;
        TickType_t last_wake = xTaskGetTickCount();
        canNode::filterInit(&hcan2);

        djiMotor c610(&hcan2, 0x200, 0x201, )
        for (;;)
        {

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kMotorCtrlPeriodMs));

        }
}
