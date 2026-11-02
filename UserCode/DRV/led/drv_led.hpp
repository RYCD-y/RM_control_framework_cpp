//
// RGB LED（三路 PWM）驱动
// 对应 control_framework_v2/UserCode/DRV/led/drv_led.{h,c}，改为 C++ 类。
//
#ifndef THREE_FRICTION_DRV_LED_HPP
#define THREE_FRICTION_DRV_LED_HPP

#include <stdint.h>

#include "stm32f4xx_hal.h"

class Led
{
        public:
                /// 构造只绑定外设，不碰硬件（安全：可全局/静态定义）
                /// 通道默认值按 DJI_C 开发板接线：R→CH3, G→CH2, B→CH1
                Led(TIM_HandleTypeDef *htim,
                    uint32_t            r_channel = TIM_CHANNEL_3,
                    uint32_t            g_channel = TIM_CHANNEL_2,
                    uint32_t            b_channel = TIM_CHANNEL_1);

                void init();                                   // 启动三路 PWM（须在 MX_TIMx_Init 之后）
                void set(uint8_t r, uint8_t g, uint8_t b);     // 0~255，要求 TIM ARR = 255
                void off() { set(0, 0, 0); }

                uint8_t red() const { return r_; }
                uint8_t green() const { return g_; }
                uint8_t blue() const { return b_; }

        private:
                TIM_HandleTypeDef *htim_      = nullptr;
                uint32_t           r_channel_ = TIM_CHANNEL_3;
                uint32_t           g_channel_ = TIM_CHANNEL_2;
                uint32_t           b_channel_ = TIM_CHANNEL_1;

                uint8_t r_ = 0;
                uint8_t g_ = 0;
                uint8_t b_ = 0;
};

#endif //THREE_FRICTION_DRV_LED_HPP
