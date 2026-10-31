//
// Created by ye on 2026/10/30.
//

#include "ent_main.hpp"

#include "debug.hpp"
#include "hal_can.hpp"

void ent_main(void)
{
        // CAN 启动顺序：MX_CANx_Init()（main.c 里已完成）
        //   → HAL_CAN_ConfigFilter() → HAL_CAN_Start()
        // 顺序错了会收不到任何帧（过滤器必须在启动前配好）。
        if (canNode::filterInit(&hcan1) == HAL_OK)
        {
                (void) HAL_CAN_Start(&hcan1);
        }
        if (canNode::filterInit(&hcan2) == HAL_OK)
        {
                (void) HAL_CAN_Start(&hcan2);
        }

        xTaskCreate(debugTask, "DeBug", 512, NULL, 5, NULL);
}
