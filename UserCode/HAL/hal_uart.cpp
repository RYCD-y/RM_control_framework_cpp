#include "hal_uart.hpp"

namespace {
std::array<UartBus*, 3> g_uart_buses{};
constexpr TickType_t kTxTimeoutTicks = pdMS_TO_TICKS(20U);
}  // namespace

UartBus::UartBus(UART_HandleTypeDef* huart) : huart_(huart) {
    if (huart_ == nullptr) return;
    for (UartBus*& bus : g_uart_buses) {
        if (bus == nullptr) {
            bus = this;
            return;
        }
    }
    huart_ = nullptr;
}

UartBus::~UartBus() {
    if (started_) return;
    for (UartBus* other : g_uart_buses) {
        if (other != nullptr && other != this && other->huart_ == huart_) return;
    }
    for (UartBus*& bus : g_uart_buses) {
        if (bus == this) bus = nullptr;
    }
}

UartBus* UartBus::Find(UART_HandleTypeDef* huart) {
    for (UartBus* bus : g_uart_buses) {
        if (bus != nullptr && bus->huart_ == huart) return bus;
    }
    return nullptr;
}

HAL_StatusTypeDef UartBus::RegisterCallback(RxCallback rx_callback, ErrorCallback error_callback,
                                            void* context) {
    if (started_ || context == nullptr || (rx_callback == nullptr && error_callback == nullptr))
        return HAL_ERROR;
    for (const CallbackEntry& entry : callbacks_) {
        if (entry.context == context) return HAL_ERROR;
    }
    for (CallbackEntry& entry : callbacks_) {
        if (entry.context == nullptr) {
            entry = {rx_callback, error_callback, context};
            return HAL_OK;
        }
    }
    return HAL_BUSY;
}

HAL_StatusTypeDef UartBus::UnregisterCallback(void* context) {
    if (started_ || context == nullptr) return HAL_ERROR;
    for (CallbackEntry& entry : callbacks_) {
        if (entry.context == context) {
            entry = {};
            return HAL_OK;
        }
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef UartBus::Start() {
    if (huart_ == nullptr || Find(huart_) != this || huart_->hdmarx == nullptr ||
        huart_->hdmatx == nullptr || huart_->hdmarx->Init.Mode != DMA_CIRCULAR ||
        xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
        return HAL_ERROR;
    if (started_) return HAL_OK;

    rx_stream_buffer_ = xStreamBufferCreateStatic(kRxStreamSize, 1U, rx_stream_storage_.data(),
                                                  &rx_stream_buffer_storage_);
    tx_message_buffer_ = xMessageBufferCreateStatic(kTxMessageBufferSize, tx_storage_.data(),
                                                    &tx_message_buffer_storage_);
    tx_complete_semaphore_ = xSemaphoreCreateBinaryStatic(&tx_complete_storage_);
    tx_mutex_ = xSemaphoreCreateMutexStatic(&tx_mutex_storage_);
    if (rx_stream_buffer_ == nullptr || tx_message_buffer_ == nullptr ||
        tx_complete_semaphore_ == nullptr || tx_mutex_ == nullptr)
        return HAL_ERROR;

    rx_task_ = xTaskCreateStatic(RxTaskEntry, "UartRx", kTaskStackSize, this, kTaskPriority,
                                 rx_task_stack_.data(), &rx_task_storage_);
    tx_task_ = xTaskCreateStatic(TxTaskEntry, "UartTx", kTaskStackSize, this, kTaskPriority,
                                 tx_task_stack_.data(), &tx_task_storage_);
    if (rx_task_ == nullptr || tx_task_ == nullptr) {
        if (rx_task_ != nullptr) vTaskDelete(rx_task_);
        if (tx_task_ != nullptr) vTaskDelete(tx_task_);
        rx_task_ = nullptr;
        tx_task_ = nullptr;
        return HAL_ERROR;
    }

    if (HAL_UART_RegisterRxEventCallback(huart_, RxEventThunk) != HAL_OK) goto rollback_tasks;
    if (HAL_UART_RegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID, TxCompleteThunk) != HAL_OK) {
        (void)HAL_UART_UnRegisterRxEventCallback(huart_);
        goto rollback_tasks;
    }
    if (HAL_UART_RegisterCallback(huart_, HAL_UART_ERROR_CB_ID, ErrorThunk) != HAL_OK) {
        (void)HAL_UART_UnRegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID);
        (void)HAL_UART_UnRegisterRxEventCallback(huart_);
        goto rollback_tasks;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_dma_buffer_.data(), rx_dma_buffer_.size()) ==
        HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
        rx_produced_position_ = 0U;
        return HAL_OK;
    }

    started_ = false;
    (void)HAL_UART_UnRegisterCallback(huart_, HAL_UART_ERROR_CB_ID);
    (void)HAL_UART_UnRegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID);
    (void)HAL_UART_UnRegisterRxEventCallback(huart_);
    goto rollback_tasks;
rollback_tasks:
    vTaskDelete(rx_task_);
    vTaskDelete(tx_task_);
    rx_task_ = nullptr;
    tx_task_ = nullptr;
    return HAL_ERROR;
}

HAL_StatusTypeDef UartBus::Send(const uint8_t* data, size_t size) {
    if (!started_ || data == nullptr || size == 0U || size > kTxMessageSize) return HAL_ERROR;
    if (xSemaphoreTake(tx_mutex_, 0U) != pdTRUE) return HAL_BUSY;
    const size_t written = xMessageBufferSend(tx_message_buffer_, data, size, 0U);
    (void)xSemaphoreGive(tx_mutex_);
    return written == size ? HAL_OK : HAL_BUSY;
}

void UartBus::RxEventThunk(UART_HandleTypeDef* huart, uint16_t size) {
    UartBus* bus = Find(huart);
    if (bus != nullptr && bus->started_) bus->OnRxEvent(size);
}

void UartBus::TxCompleteThunk(UART_HandleTypeDef* huart) {
    UartBus* bus = Find(huart);
    if (bus != nullptr && bus->started_) bus->OnTxComplete();
}

void UartBus::ErrorThunk(UART_HandleTypeDef* huart) {
    UartBus* bus = Find(huart);
    if (bus != nullptr && bus->started_) bus->OnError();
}

void UartBus::RxTaskEntry(void* context) { static_cast<UartBus*>(context)->RxTask(); }
void UartBus::TxTaskEntry(void* context) { static_cast<UartBus*>(context)->TxTask(); }

void UartBus::OnRxEvent(uint16_t size) {
    const uint32_t previous = rx_produced_position_;
    const uint32_t position = (kRxDmaSize - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % kRxDmaSize;
    const HAL_UART_RxEventTypeTypeDef event_type = HAL_UARTEx_GetRxEventType(huart_);
    if (position == previous && event_type != HAL_UART_RXEVENT_TC) return;

    const uint32_t count =
        position == previous
            ? kRxDmaSize
            : (position > previous ? position - previous : kRxDmaSize - previous + position);
    rx_produced_position_ = position;
    if (count == 0U) return;

    BaseType_t task_woken = pdFALSE;
    size_t sent = 0U;
    UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    if (count == kRxDmaSize) {
        sent =
            xStreamBufferSendFromISR(rx_stream_buffer_, rx_dma_buffer_.data(), count, &task_woken);
    } else {
        const size_t remaining = static_cast<size_t>(kRxDmaSize - previous);
        const size_t first_size = count < remaining ? count : remaining;
        sent += xStreamBufferSendFromISR(rx_stream_buffer_, rx_dma_buffer_.data() + previous,
                                         first_size, &task_woken);
        const size_t second_size = count - first_size;
        if (second_size != 0U) {
            sent += xStreamBufferSendFromISR(rx_stream_buffer_, rx_dma_buffer_.data(), second_size,
                                             &task_woken);
        }
    }
    taskEXIT_CRITICAL_FROM_ISR(mask);
    rx_dropped_bytes_ += static_cast<uint32_t>(count - sent);
    if (sent != count) rx_restart_pending_ = true;
    if (rx_restart_pending_) vTaskNotifyGiveFromISR(rx_task_, &task_woken);
    portYIELD_FROM_ISR(task_woken);
    (void)size;
}

void UartBus::OnTxComplete() {
    BaseType_t task_woken = pdFALSE;
    (void)xSemaphoreGiveFromISR(tx_complete_semaphore_, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}

void UartBus::OnError() {
    ++rx_errors_;
    rx_restart_pending_ = true;
    BaseType_t task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(rx_task_, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}

void UartBus::RxTask() {
    std::array<uint8_t, 64> data{};
    for (;;) {
        const size_t size =
            xStreamBufferReceive(rx_stream_buffer_, data.data(), data.size(), pdMS_TO_TICKS(2U));
        if (size != 0U) {
            for (const CallbackEntry& entry : callbacks_) {
                if (entry.context != nullptr && entry.rx_callback != nullptr)
                    entry.rx_callback(entry.context, data.data(), size);
            }
        }
        if (ulTaskNotifyTake(pdTRUE, 0U) != 0U || rx_restart_pending_) {
            rx_restart_pending_ = false;
            (void)xStreamBufferReset(rx_stream_buffer_);
            const HAL_StatusTypeDef status = RestartRx();
            if (status != HAL_OK) {
                ++rx_errors_;
                NotifyErrors(huart_->ErrorCode);
                vTaskDelay(pdMS_TO_TICKS(1U));
                rx_restart_pending_ = true;
            }
        }
    }
}

void UartBus::TxTask() {
    for (;;) {
        const size_t size = xMessageBufferReceive(tx_message_buffer_, tx_dma_buffer_.data(),
                                                  tx_dma_buffer_.size(), portMAX_DELAY);
        if (size == 0U) continue;
        (void)xSemaphoreTake(tx_complete_semaphore_, 0U);
        if (HAL_UART_Transmit_DMA(huart_, tx_dma_buffer_.data(), size) != HAL_OK ||
            xSemaphoreTake(tx_complete_semaphore_, kTxTimeoutTicks) != pdTRUE) {
            (void)HAL_UART_AbortTransmit(huart_);
            (void)xSemaphoreTake(tx_complete_semaphore_, 0U);
            ++tx_errors_;
            NotifyErrors(huart_->ErrorCode);
        }
    }
}

HAL_StatusTypeDef UartBus::RestartRx() {
    if (huart_ == nullptr || huart_->hdmarx == nullptr || huart_->hdmarx->Init.Mode != DMA_CIRCULAR)
        return HAL_ERROR;
    // Stop the previous transfer first; otherwise HAL keeps RxState busy and every retry fails.
    (void)HAL_UART_AbortReceive(huart_);
    const HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_dma_buffer_.data(), rx_dma_buffer_.size());
    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
        rx_produced_position_ = 0U;
    }
    return status;
}

void UartBus::NotifyErrors(uint32_t error_code) {
    for (const CallbackEntry& entry : callbacks_) {
        if (entry.context != nullptr && entry.error_callback != nullptr)
            entry.error_callback(entry.context, error_code);
    }
}
