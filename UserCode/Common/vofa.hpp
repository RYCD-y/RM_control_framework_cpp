//
// VOFA+ 上位机调试输出（JustFloat 协议）
// 全局唯一的调试外设 → 用 namespace + 自由函数，不做成类。
//
#ifndef THREE_FRICTION_VOFA_HPP
#define THREE_FRICTION_VOFA_HPP

#include <stdint.h>

#include "stm32f4xx_hal.h"

namespace vofa
{
        inline constexpr uint8_t kChannelCount = 12;   // 每次发送 12 个 float

        /// 绑定调试串口：注册空闲/错误回调并启动空闲中断 DMA 接收
        /// 必须在 MX_USARTx_UART_Init() 之后、多任务并发之前调用（如 ent_main）
        void init(UART_HandleTypeDef *huart);

        /// 只写通道缓冲，不发送（与 flush() 配对使用，保证同一帧里各通道同源）
        void set(uint8_t channel, float value);

        /// 尝试把当前缓冲整帧发出（内置 kTxDelayMs 节流）
        /// 返回 HAL_OK 表示已发起 DMA，HAL_BUSY 表示被节流或串口忙
        HAL_StatusTypeDef flush();

        /// 便捷接口 = set() + flush()（多通道时建议改用 set/flush）
        HAL_StatusTypeDef send(uint8_t channel, float value);

        uint32_t errorCount();
} // namespace vofa

#endif //THREE_FRICTION_VOFA_HPP
