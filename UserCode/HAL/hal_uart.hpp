#ifndef THREE_FRICTION_HAL_UART_HPP
#define THREE_FRICTION_HAL_UART_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"
#include "stream_buffer.h"
#include "task.h"

class UartBus {
   public:
    static constexpr size_t kRxDmaSize = 128U;
    static constexpr size_t kRxStreamSize = 512U;
    static constexpr size_t kTxMessageSize = 128U;
    static constexpr size_t kTxMessageBufferSize = 1024U;
    static constexpr size_t kCallbackCount = 4U;
    static constexpr uint32_t kTaskStackSize = 256U;
    static constexpr UBaseType_t kTaskPriority = 4U;

    using RxCallback = void (*)(void* context, const uint8_t* data, size_t size);
    using ErrorCallback = void (*)(void* context, uint32_t error_code);

    explicit UartBus(UART_HandleTypeDef* huart);
    ~UartBus();

    UartBus(const UartBus&) = delete;
    UartBus& operator=(const UartBus&) = delete;

    HAL_StatusTypeDef RegisterCallback(RxCallback rx_callback, ErrorCallback error_callback,
                                       void* context);
    HAL_StatusTypeDef UnregisterCallback(void* context);
    HAL_StatusTypeDef Start();
    HAL_StatusTypeDef Send(const uint8_t* data, size_t size);
    bool IsStarted() const { return started_; }
    uint32_t RxDroppedBytes() const { return rx_dropped_bytes_; }
    uint32_t TxErrors() const { return tx_errors_; }
    uint32_t RxErrors() const { return rx_errors_; }

   private:
    struct CallbackEntry {
        RxCallback rx_callback = nullptr;
        ErrorCallback error_callback = nullptr;
        void* context = nullptr;
    };

    static constexpr size_t kMaxBuses = 3U;

    static UartBus* Find(UART_HandleTypeDef* huart);
    static void RxEventThunk(UART_HandleTypeDef* huart, uint16_t size);
    static void TxCompleteThunk(UART_HandleTypeDef* huart);
    static void ErrorThunk(UART_HandleTypeDef* huart);
    static void RxTaskEntry(void* context);
    static void TxTaskEntry(void* context);

    void OnRxEvent(uint16_t size);
    void OnTxComplete();
    void OnError();
    void RxTask();
    void TxTask();
    HAL_StatusTypeDef RestartRx();
    void NotifyErrors(uint32_t error_code);

    UART_HandleTypeDef* huart_ = nullptr;
    std::array<uint8_t, kRxDmaSize> rx_dma_buffer_{};
    std::array<uint8_t, kTxMessageSize> tx_dma_buffer_{};
    std::array<CallbackEntry, kCallbackCount> callbacks_{};
    std::array<uint8_t, kTxMessageBufferSize> tx_storage_{};
    StaticMessageBuffer_t tx_message_buffer_storage_{};
    StaticStreamBuffer_t rx_stream_buffer_storage_{};
    std::array<uint8_t, kRxStreamSize> rx_stream_storage_{};
    StaticSemaphore_t tx_complete_storage_{};
    StaticSemaphore_t tx_mutex_storage_{};
    StaticTask_t rx_task_storage_{};
    StaticTask_t tx_task_storage_{};
    std::array<StackType_t, kTaskStackSize> rx_task_stack_{};
    std::array<StackType_t, kTaskStackSize> tx_task_stack_{};
    MessageBufferHandle_t tx_message_buffer_ = nullptr;
    StreamBufferHandle_t rx_stream_buffer_ = nullptr;
    SemaphoreHandle_t tx_complete_semaphore_ = nullptr;
    SemaphoreHandle_t tx_mutex_ = nullptr;
    TaskHandle_t rx_task_ = nullptr;
    TaskHandle_t tx_task_ = nullptr;
    volatile uint32_t rx_produced_position_ = 0U;
    volatile uint32_t rx_dropped_bytes_ = 0U;
    volatile uint32_t tx_errors_ = 0U;
    volatile uint32_t rx_errors_ = 0U;
    volatile bool rx_restart_pending_ = false;
    bool started_ = false;
};

#endif  // THREE_FRICTION_HAL_UART_HPP
