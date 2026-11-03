//
// VOFA+ 上位机调试输出（JustFloat 协议）
// 全局唯一的调试外设 → 用 namespace + 自由函数，不做成类。
//
#ifndef THREE_FRICTION_VOFA_HPP
#define THREE_FRICTION_VOFA_HPP

#include <stdint.h>

#include "stm32f4xx_hal.h"

namespace vofa {
inline constexpr uint8_t kChannelCount = 12;
void Init(UART_HandleTypeDef* huart);
void Set(uint8_t channel, float value);
HAL_StatusTypeDef Flush();
HAL_StatusTypeDef Send(uint8_t channel, float value);
uint32_t ErrorCount();
}  // namespace vofa

#endif  // THREE_FRICTION_VOFA_HPP
