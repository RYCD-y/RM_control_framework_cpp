#include "hal_uart.hpp"

namespace
{
        // 全局总线注册表：供 HAL 静态回调按句柄反查实例
        std::array<UartBus *, 3> g_uart_buses{};
        // 单条报文的 DMA 发送完成等待上限
        constexpr TickType_t     kTxTimeoutTicks = pdMS_TO_TICKS(20U);
} // namespace

// 绑定 UART 句柄并登记；同一句柄只允许一个实例
UartBus::UartBus(UART_HandleTypeDef *huart) : huart_(huart)
{
        if (huart_ == nullptr) return;
        for (UartBus *&bus : g_uart_buses)
        {
                if (bus == nullptr)
                {
                        bus = this;
                        return;
                }
        }
        huart_ = nullptr;
}

// 未启动时从注册表移除，允许重新构造
UartBus::~UartBus()
{
        if (started_) return;
        for (UartBus *other : g_uart_buses)
        {
                if (other != nullptr && other != this && other->huart_ == huart_) return;
        }
        for (UartBus *&bus : g_uart_buses)
        {
                if (bus == this) bus = nullptr;
        }
}

// 按句柄查找已注册的总线
UartBus *UartBus::Find(UART_HandleTypeDef *huart)
{
        for (UartBus *bus : g_uart_buses)
        {
                if (bus != nullptr && bus->huart_ == huart) return bus;
        }
        return nullptr;
}

// 登记接收/错误回调，容量满则 Busy
HAL_StatusTypeDef UartBus::RegisterCallback(RxCallback rx_callback, ErrorCallback error_callback,
                                            void *     context)
{
        if (started_ || context == nullptr || (rx_callback == nullptr && error_callback == nullptr))
                return HAL_ERROR;
        for (const CallbackEntry &entry : callbacks_)
        {
                if (entry.context == context) return HAL_ERROR;
        }
        for (CallbackEntry &entry : callbacks_)
        {
                if (entry.context == nullptr)
                {
                        entry = {rx_callback, error_callback, context};
                        return HAL_OK;
                }
        }
        return HAL_BUSY;
}

// 按 context 注销回调
HAL_StatusTypeDef UartBus::UnregisterCallback(void *context)
{
        if (started_ || context == nullptr) return HAL_ERROR;
        for (CallbackEntry &entry : callbacks_)
        {
                if (entry.context == context)
                {
                        entry = {};
                        return HAL_OK;
                }
        }
        return HAL_ERROR;
}

// 创建静态缓冲与任务、注册 HAL 回调并启动接收
HAL_StatusTypeDef UartBus::Start()
{
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
        tx_mutex_              = xSemaphoreCreateMutexStatic(&tx_mutex_storage_);
        if (rx_stream_buffer_ == nullptr || tx_message_buffer_ == nullptr ||
            tx_complete_semaphore_ == nullptr || tx_mutex_ == nullptr)
                return HAL_ERROR;

        rx_task_ = xTaskCreateStatic(RxTaskEntry, "UartRx", kTaskStackSize, this, kTaskPriority,
                                     rx_task_stack_.data(), &rx_task_storage_);
        tx_task_ = xTaskCreateStatic(TxTaskEntry, "UartTx", kTaskStackSize, this, kTaskPriority,
                                     tx_task_stack_.data(), &tx_task_storage_);
        if (rx_task_ == nullptr || tx_task_ == nullptr) return RollbackStart();

        if (HAL_UART_RegisterRxEventCallback(huart_, RxEventThunk) != HAL_OK) return RollbackStart();
        if (HAL_UART_RegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID, TxCompleteThunk) != HAL_OK)
                return RollbackStart();
        if (HAL_UART_RegisterCallback(huart_, HAL_UART_ERROR_CB_ID, ErrorThunk) != HAL_OK)
                return RollbackStart();

        // 先置 started_ 再开接收：否则启动瞬间到达的数据会被 ISR 当作"未启动"丢弃。
        started_ = true;
        if (HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_dma_buffer_.data(), rx_dma_buffer_.size()) !=
            HAL_OK)
                return RollbackStart();

        __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
        rx_produced_position_ = 0U;
        return HAL_OK;
}

// 启动失败回滚：清状态、注销 HAL 回调、删除已建任务；靠 nullptr 检查保证可重复调用
HAL_StatusTypeDef UartBus::RollbackStart()
{
        started_ = false;
        (void) HAL_UART_UnRegisterCallback(huart_, HAL_UART_ERROR_CB_ID);
        (void) HAL_UART_UnRegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID);
        (void) HAL_UART_UnRegisterRxEventCallback(huart_);
        if (rx_task_ != nullptr) vTaskDelete(rx_task_);
        if (tx_task_ != nullptr) vTaskDelete(tx_task_);
        rx_task_ = nullptr;
        tx_task_ = nullptr;
        return HAL_ERROR;
}

// 整帧入队；加锁保护多写者，入队失败返回 Busy
HAL_StatusTypeDef UartBus::Send(const uint8_t *data, size_t size)
{
        if (!started_ || data == nullptr || size == 0U || size > kTxMessageSize) return HAL_ERROR;
        if (xSemaphoreTake(tx_mutex_, 0U) != pdTRUE) return HAL_BUSY;
        const size_t written = xMessageBufferSend(tx_message_buffer_, data, size, 0U);
        (void) xSemaphoreGive(tx_mutex_);
        return written == size ? HAL_OK : HAL_BUSY;
}

// HAL 接收事件中转：按句柄找回实例后交给 ISR 处理
void UartBus::RxEventThunk(UART_HandleTypeDef *huart, uint16_t size)
{
        UartBus *bus = Find(huart);
        if (bus != nullptr && bus->started_) bus->OnRxEvent(size);
}

// HAL 发送完成中转
void UartBus::TxCompleteThunk(UART_HandleTypeDef *huart)
{
        UartBus *bus = Find(huart);
        if (bus != nullptr && bus->started_) bus->OnTxComplete();
}

// HAL 错误中转
void UartBus::ErrorThunk(UART_HandleTypeDef *huart)
{
        UartBus *bus = Find(huart);
        if (bus != nullptr && bus->started_) bus->OnError();
}

// FreeRTOS 接收任务入口
void UartBus::RxTaskEntry(void *context) { static_cast<UartBus *>(context)->RxTask(); }
// FreeRTOS 发送任务入口
void UartBus::TxTaskEntry(void *context) { static_cast<UartBus *>(context)->TxTask(); }

// ISR：用 DMA 剩余计数算出新增字节区间，回卷时分段搬运
void UartBus::OnRxEvent(uint16_t size)
{
        const uint32_t                    previous   = rx_produced_position_;
        const uint32_t                    position   = (kRxDmaSize - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % kRxDmaSize;
        const HAL_UART_RxEventTypeTypeDef event_type = HAL_UARTEx_GetRxEventType(huart_);
        if (position == previous && event_type != HAL_UART_RXEVENT_TC) return;

        const uint32_t count =
                        position == previous
                                ? kRxDmaSize
                                : (position > previous ? position - previous : kRxDmaSize - previous + position);
        rx_produced_position_ = position;
        if (count == 0U) return;

        BaseType_t  task_woken = pdFALSE;
        size_t      sent       = 0U;
        UBaseType_t mask       = taskENTER_CRITICAL_FROM_ISR();
        if (count == kRxDmaSize)
        {
                sent =
                                xStreamBufferSendFromISR(rx_stream_buffer_, rx_dma_buffer_.data(), count, &task_woken);
        }
        else
        {
                const size_t remaining  = static_cast<size_t>(kRxDmaSize - previous);
                const size_t first_size = count < remaining ? count : remaining;
                sent                    += xStreamBufferSendFromISR(rx_stream_buffer_, rx_dma_buffer_.data() + previous,
                                                 first_size, &task_woken);
                const size_t second_size = count - first_size;
                if (second_size != 0U)
                {
                        sent += xStreamBufferSendFromISR(rx_stream_buffer_, rx_dma_buffer_.data(), second_size,
                                                         &task_woken);
                }
        }
        taskEXIT_CRITICAL_FROM_ISR(mask);
        rx_dropped_bytes_ += static_cast<uint32_t>(count - sent);
        // 只置位标志，不唤醒任务：RX 任务已阻塞在 StreamBuffer 上，
        // 而 StreamBuffer 内部就用任务通知等待，再从 ISR 发通知会与其争用同一通知槽。
        if (sent != count) rx_restart_pending_ = true;
        portYIELD_FROM_ISR(task_woken);
        (void) size;
}

// ISR：通知 TX 任务本帧发完
void UartBus::OnTxComplete()
{
        BaseType_t task_woken = pdFALSE;
        (void) xSemaphoreGiveFromISR(tx_complete_semaphore_, &task_woken);
        portYIELD_FROM_ISR(task_woken);
}

// ISR：仅标记接收需重启；由 RX 任务在超时轮询中处理
void UartBus::OnError()
{
        ++rx_errors_;
        rx_restart_pending_ = true;
}

// RX 任务：把字节递交给各注册回调；需要时重置并重启接收
void UartBus::RxTask()
{
        std::array<uint8_t, 64> data{};
        for (;;)
        {
                const size_t size =
                                xStreamBufferReceive(rx_stream_buffer_, data.data(), data.size(), pdMS_TO_TICKS(2U));
                if (size != 0U)
                {
                        for (const CallbackEntry &entry : callbacks_)
                        {
                                if (entry.context != nullptr && entry.rx_callback != nullptr)
                                        entry.rx_callback(entry.context, data.data(), size);
                        }
                }
                // 只看标志位：不使用任务通知，避免与 StreamBuffer 内部等待争用通知槽。
                // 代价是重启最多延迟一个 2ms 超时周期，属错误恢复路径，可接受。
                if (rx_restart_pending_)
                {
                        rx_restart_pending_            = false;
                        const HAL_StatusTypeDef status = RestartRx();
                        if (status != HAL_OK)
                        {
                                ++rx_errors_;
                                NotifyErrors(huart_->ErrorCode);
                                vTaskDelay(pdMS_TO_TICKS(1U));
                                rx_restart_pending_ = true;
                        }
                }
        }
}

// TX 任务：逐条取出完整报文，DMA 发送并等完成，超时则中止
void UartBus::TxTask()
{
        for (;;)
        {
                const size_t size = xMessageBufferReceive(tx_message_buffer_, tx_dma_buffer_.data(),
                                                          tx_dma_buffer_.size(), portMAX_DELAY);
                if (size == 0U) continue;
                (void) xSemaphoreTake(tx_complete_semaphore_, 0U);
                if (HAL_UART_Transmit_DMA(huart_, tx_dma_buffer_.data(), size) != HAL_OK ||
                    xSemaphoreTake(tx_complete_semaphore_, kTxTimeoutTicks) != pdTRUE)
                {
                        (void) HAL_UART_AbortTransmit(huart_);
                        (void) xSemaphoreTake(tx_complete_semaphore_, 0U);
                        ++tx_errors_;
                        NotifyErrors(huart_->ErrorCode);
                }
        }
}

// 中止旧传输 → 清残帧 → 重启循环 DMA 接收
HAL_StatusTypeDef UartBus::RestartRx()
{
        if (huart_ == nullptr || huart_->hdmarx == nullptr || huart_->hdmarx->Init.Mode != DMA_CIRCULAR)
                return HAL_ERROR;
        // 先停 DMA：此后不会再有 RxEvent ISR 往 StreamBuffer 写，
        // 再 reset 才不会与 ISR 并发导致数据错乱。
        (void) HAL_UART_AbortReceive(huart_);
        (void) xStreamBufferReset(rx_stream_buffer_);
        const HAL_StatusTypeDef status =
                        HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_dma_buffer_.data(), rx_dma_buffer_.size());
        if (status == HAL_OK)
        {
                __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
                rx_produced_position_ = 0U;
        }
        return status;
}

// 在任务上下文向注册者通报错误码
void UartBus::NotifyErrors(uint32_t error_code)
{
        for (const CallbackEntry &entry : callbacks_)
        {
                if (entry.context != nullptr && entry.error_callback != nullptr)
                        entry.error_callback(entry.context, error_code);
        }
}
