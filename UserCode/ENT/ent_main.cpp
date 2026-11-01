//
// Created by ye on 2026/10/30.
//

#include "ent_main.hpp"

#include "debug.hpp"
#include "dwt.hpp"
#include "hal_can.hpp"
#include "usart.h"
#include "vofa.hpp"

void ent_main(void)
{
        Dwt::init(168);

        // CAN 启动顺序：MX_CANx_Init()（main.c 里已完成）
        //   → HAL_CAN_ConfigFilter() → HAL_CAN_Start()
        // 顺序错了会收不到任何帧（过滤器必须在启动前配好）。
        if (CanNode::filterInit(&hcan1) == HAL_OK)
        {
                (void) HAL_CAN_Start(&hcan1);
        }
        if (CanNode::filterInit(&hcan2) == HAL_OK)
        {
                (void) HAL_CAN_Start(&hcan2);
        }

        // VOFA 调试串口：必须在 MX_USART6_UART_Init 之后、任务并发之前初始化
        vofa::init(&huart6);



        xTaskCreate(debugTask, "DeBug", 512, nullptr, 5, nullptr);
}
