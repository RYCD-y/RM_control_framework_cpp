//
// Created by ye on 2026/10/30.
//

#include "ent_main.hpp"

#include "debug.hpp"
#include "dwt.hpp"
#include "ent_devices.hpp"
#include "hal_can.hpp"
#include "tim.h"
#include "usart.h"
#include "vofa.hpp"

// 全局外设对象：构造只绑定句柄，不碰硬件
Led led(&htim5);
Buzzer buzzer(&htim4, TIM_CHANNEL_3);

void ent_main(void) {
    Dwt::Init(168);

    // CAN 启动顺序：MX_CANx_Init()（main.c 里已完成）
    //   → HAL_CAN_ConfigFilter() → HAL_CAN_Start()
    // 顺序错了会收不到任何帧（过滤器必须在启动前配好）。
    if (CanNode::FilterInit(&hcan1) == HAL_OK) {
        (void)HAL_CAN_Start(&hcan1);
    }
    if (CanNode::FilterInit(&hcan2) == HAL_OK) {
        (void)HAL_CAN_Start(&hcan2);
    }

    // VOFA 调试串口：必须在 MX_USART6_UART_Init 之后、任务并发之前初始化
    vofa::Init(&huart6);

    // LED（TIM5, ARR=255）：启动 PWM
    led.Init();

    // 蜂鸣器（TIM4 CH3）：启动 PWM + 创建音效播放任务（内部完成）
    (void)buzzer.Init("Buzzer");

    led.Set(0, 255, 0);  // 绿灯：启动完成
    buzzer.SetSoundEffect(Buzzer::SoundEffect::SystemStart);

    xTaskCreate(DebugTask, "DeBug", 512, nullptr, 5, nullptr);
}
