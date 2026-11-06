//
// VOFA+ 上位机调试输出（JustFloat 协议）
// 全局唯一的调试外设 → 用 namespace + 自由函数，不做成类。
//
#ifndef THREE_FRICTION_VOFA_HPP
#define THREE_FRICTION_VOFA_HPP

#include <stdint.h>

// pi-lens-ignore: unused-include
#include "stm32f4xx_hal.h"

namespace vofa {
// JustFloat 每帧固定 12 个 float
inline constexpr uint8_t kChannelCount = 12;
// 绑定调试串口并启动统一 UART 通道（需在串口初始化之后调用）
void Init(UART_HandleTypeDef* huart);
// 只写通道缓冲，不发送
void Set(uint8_t channel, float value);
// 尝试整帧发送（内置节流），HAL_BUSY 表示被节流或串口忙
HAL_StatusTypeDef Flush();
// 便捷接口 = Set + Flush
HAL_StatusTypeDef Send(uint8_t channel, float value);
// UART 错误计数（调试用）
uint32_t ErrorCount();
}  // namespace vofa

#endif  // THREE_FRICTION_VOFA_HPP
