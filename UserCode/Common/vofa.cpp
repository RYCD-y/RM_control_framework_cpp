#include "vofa.hpp"

#include <array>
#include <cstring>

#include "dwt.hpp"
#include "hal_uart.hpp"
#include "usart.h"

namespace {
constexpr size_t kFrameSize = vofa::kChannelCount * sizeof(float) + 4U;
constexpr float kTxDelayMs = 20.0f;

struct VofaState {
    UartBus uart{&huart6};

    std::array<float, vofa::kChannelCount> channels{};
    std::array<uint8_t, kFrameSize> frame{};
    float last_tx_time_ms = -kTxDelayMs;
    uint32_t error_count = 0U;
};

VofaState g_vofa;

void OnUartError(void* context, uint32_t error_code) {
    (void)error_code;
    ++static_cast<VofaState*>(context)->error_count;
}
}  // namespace

namespace vofa {
void Init(UART_HandleTypeDef* huart) {
    if (huart != &huart6 || g_vofa.uart.IsStarted()) return;
    constexpr uint8_t kTail[4] = {0x00, 0x00, 0x80, 0x7F};
    std::memcpy(g_vofa.frame.data() + kChannelCount * sizeof(float), kTail, sizeof(kTail));
    if (g_vofa.uart.RegisterCallback(nullptr, OnUartError, &g_vofa) != HAL_OK) return;
    (void)g_vofa.uart.Start();
}

void Set(uint8_t channel, float value) {
    if (channel < kChannelCount) g_vofa.channels[channel] = value;
}

HAL_StatusTypeDef Flush() {
    if (!g_vofa.uart.IsStarted()) return HAL_ERROR;
    const float now = Dwt::TimelineMs();
    if (now - g_vofa.last_tx_time_ms <= kTxDelayMs) return HAL_BUSY;
    std::memcpy(g_vofa.frame.data(), g_vofa.channels.data(), kChannelCount * sizeof(float));
    const HAL_StatusTypeDef status = g_vofa.uart.Send(g_vofa.frame.data(), g_vofa.frame.size());
    if (status == HAL_OK) g_vofa.last_tx_time_ms = now;
    return status;
}

HAL_StatusTypeDef Send(uint8_t channel, float value) {
    if (channel >= kChannelCount) return HAL_ERROR;
    Set(channel, value);
    return Flush();
}

uint32_t ErrorCount() { return g_vofa.error_count; }
}  // namespace vofa
