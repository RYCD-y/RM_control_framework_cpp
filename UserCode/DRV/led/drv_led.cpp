//
// RGB LED 驱动实现
//
#include "led/drv_led.hpp"

Led::Led(TIM_HandleTypeDef *htim, uint32_t r_channel, uint32_t g_channel, uint32_t b_channel)
        : htim_(htim),
          r_channel_(r_channel),
          g_channel_(g_channel),
          b_channel_(b_channel)
{
}

void Led::init()
{
        if (htim_ == nullptr) { return; }

        // 三路都用正向通道
        (void) HAL_TIM_PWM_Start(htim_, r_channel_);
        (void) HAL_TIM_PWM_Start(htim_, g_channel_);
        (void) HAL_TIM_PWM_Start(htim_, b_channel_);
}

void Led::set(uint8_t r, uint8_t g, uint8_t b)
{
        r_ = r;
        g_ = g;
        b_ = b;

        if (htim_ == nullptr) { return; }

        __HAL_TIM_SET_COMPARE(htim_, r_channel_, r_);
        __HAL_TIM_SET_COMPARE(htim_, g_channel_, g_);
        __HAL_TIM_SET_COMPARE(htim_, b_channel_, b_);
}
