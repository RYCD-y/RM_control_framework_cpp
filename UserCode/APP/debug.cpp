//
// Created by ye on 2026/10/30.
//

#include "debug.hpp"

#include <stdint.h>

#include <cmath>

#include "FreeRTOS.h"
#include "cotr.hpp"
#include "drv_dji_motor.hpp"
#include "ent_devices.hpp"
#include "hal_can.hpp"
#include "task.h"
#include "vofa.hpp"

// TODO:多态实验
void DebugTask(void* ptr) {
    (void)ptr;

    static constexpr TickType_t kMotorCtrlPeriodMs = 1U;

    vTaskDelay(3000);
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kMotorCtrlPeriodMs));
    }
}
