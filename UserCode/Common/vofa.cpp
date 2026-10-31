//
// VOFA+ 调试输出实现（JustFloat：12 个 float + 帧尾 0x00 0x00 0x80 0x7F）
//
#include "vofa.hpp"

#include <cstring>

#include "dwt.hpp"

namespace
{
        constexpr uint8_t kTxSize    = vofa::kChannelCount;
        constexpr uint8_t kRxSize    = 12;
        constexpr float   kTxDelayMs = 20.0f;

        // 全部为静态存储期、零初始化：没有构造顺序问题，也不需要析构
        UART_HandleTypeDef *s_huart                    = nullptr;
        float               s_tx_data[kTxSize]         = {};
        float               s_rx_data[kRxSize]         = {};
        uint8_t             s_message[kTxSize * 4 + 4] = {};
        float               s_tx_time_last             = -kTxDelayMs;
        volatile uint32_t   s_error_count              = 0;

        void rxEventCallback(UART_HandleTypeDef *huart, uint16_t pos)
        {
                (void) pos;
                if (s_huart == nullptr) { return; }

                // 重新开启下一次接收（必须在 ISR 内完成，否则丢帧）
                (void) HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *) s_rx_data, sizeof(s_rx_data));
                if (huart->hdmarx != nullptr) { __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); }

                // TODO: 需要收上位机指令时，用 pos 作为本帧长度解析 s_rx_data
        }

        void uartErrorCallback(UART_HandleTypeDef *huart)
        {
                (void) huart; // 各串口互相独立，这里只计数
                s_error_count++;
        }
} // namespace

namespace vofa
{
        void init(UART_HandleTypeDef *huart)
        {
                // JustFloat 帧尾放在最后 4 字节
                static constexpr uint8_t kTail[4] = {0x00, 0x00, 0x80, 0x7F};
                std::memcpy(s_message + kTxSize * 4, kTail, sizeof(kTail));

                s_huart = huart;
                if (huart == nullptr) { return; }

                (void) HAL_UART_RegisterRxEventCallback(huart, rxEventCallback);
                (void) HAL_UART_RegisterCallback(huart, HAL_UART_ERROR_CB_ID, uartErrorCallback);

                (void) HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *) s_rx_data, sizeof(s_rx_data));
                if (huart->hdmarx != nullptr) { __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); }
        }

        void set(uint8_t channel, float value)
        {
                if (channel < kTxSize) { s_tx_data[channel] = value; }
        }

        HAL_StatusTypeDef flush()
        {
                if (s_huart == nullptr) { return HAL_ERROR; }

                const float now = Dwt::timelineMs();
                if (now - s_tx_time_last <= kTxDelayMs) { return HAL_BUSY; }
                if (s_huart->gState != HAL_UART_STATE_READY) { return HAL_BUSY; }

                s_tx_time_last = now;

                std::memcpy(s_message, s_tx_data, sizeof(s_tx_data));
                return HAL_UART_Transmit_DMA(s_huart, s_message, sizeof(s_message));
        }

        HAL_StatusTypeDef send(uint8_t channel, float value)
        {
                if (channel >= kTxSize) { return HAL_ERROR; }

                set(channel, value);
                return flush();
        }

        uint32_t errorCount() { return s_error_count; }
} // namespace vofa
