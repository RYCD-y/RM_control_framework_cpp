#ifndef THREE_FRICTION_HAL_TIM_HPP
#define THREE_FRICTION_HAL_TIM_HPP

#include <cstdint>

#include "stm32f4xx_hal.h"

namespace tim_util {

// APB1/APB2 定时器计数时钟：APB 预分频不为 1 时，定时器时钟为 PCLK 的两倍。
uint32_t ClockFreqHz(const TIM_TypeDef* tim);

// 1us 对应的定时器计数值（读取运行期 PSC，避免与 Init 缓存不一致）。
float TicksPerUs(const TIM_HandleTypeDef* htim);

}  // namespace tim_util

#endif  // THREE_FRICTION_HAL_TIM_HPP
