#include "hal_uart.hpp"

// 绑定 UART 句柄并保存；构造不碰硬件
UartBus::UartBus(UART_HandleTypeDef *huart) : huart_(huart) {}

// 登记到注册表：同一句柄只允许一个实例，满则失败
bool UartBus::Attach(UartBus *bus)
{
        for (UartBus *const slot : buses_)
        {
                if (slot == bus) return true;
                if (slot != nullptr && slot->huart_ == bus->huart_) return false;
        }
        for (UartBus *&slot : buses_)
        {
                if (slot == nullptr)
                {
                        slot = bus;
                        return true;
                }
        }
        return false;
}

// 按句柄查找已登记的总线
UartBus *UartBus::Lookup(UART_HandleTypeDef *huart)
{
        for (UartBus *const bus : buses_)
        {
                if (bus != nullptr && bus->huart_ == huart) return bus;
        }
        return nullptr;
}

// 登记接收/错误回调；同参数重复登记视为成功，参数不同则 Busy
HAL_StatusTypeDef UartBus::RegisterCallback(RxCallback rx_callback, ErrorCallback error_callback,
                                            void *     context)
{
        if (started_ || context == nullptr || (rx_callback == nullptr && error_callback == nullptr))
                return HAL_ERROR;
        if (context_ != nullptr)
        {
                const bool same =
                                context_ == context && rx_callback_ == rx_callback && error_callback_ == error_callback;
                return same ? HAL_OK : HAL_BUSY;
        }
        rx_callback_    = rx_callback;
        error_callback_ = error_callback;
        context_        = context;
        return HAL_OK;
}

// 创建静态缓冲与任务、注册 HAL 回调并启动接收
HAL_StatusTypeDef UartBus::Start()
{
        if (huart_ == nullptr || huart_->hdmarx == nullptr || huart_->hdmatx == nullptr ||
            huart_->hdmarx->Init.Mode != DMA_NORMAL ||
            xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
                return HAL_ERROR;
        if (started_) return HAL_OK;
        if (!Attach(this)) return HAL_ERROR;

        rx_stream_buffer_ = xStreamBufferCreateStatic(kRxStreamSize, 1U, rx_stream_storage_.data(),
                                                      &rx_stream_buffer_storage_);
        tx_message_buffer_ =
                        xMessageBufferCreateStatic(kTxQueueSize, tx_storage_.data(), &tx_message_buffer_storage_);
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

        // 先置 started_ 再开接收：启动瞬间到达的事件才不会被当成"未启动"丢弃。
        started_ = true;
        if (ArmRx() != HAL_OK) return RollbackStart();
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
        if (!started_ || data == nullptr || size == 0U || size > kTxFrameSize) return HAL_ERROR;
        if (xSemaphoreTake(tx_mutex_, 0U) != pdTRUE) return HAL_BUSY;
        const size_t written = xMessageBufferSend(tx_message_buffer_, data, size, 0U);
        (void) xSemaphoreGive(tx_mutex_);
        return written == size ? HAL_OK : HAL_BUSY;
}

// HAL 接收事件中转：按句柄找回实例后交给 ISR 处理
void UartBus::RxEventThunk(UART_HandleTypeDef *huart, uint16_t size)
{
        UartBus *bus = Lookup(huart);
        if (bus != nullptr && bus->started_) bus->OnRxEvent(size);
}

// HAL 发送完成中转
void UartBus::TxCompleteThunk(UART_HandleTypeDef *huart)
{
        UartBus *bus = Lookup(huart);
        if (bus != nullptr && bus->started_) bus->OnTxComplete();
}

// HAL 错误中转
void UartBus::ErrorThunk(UART_HandleTypeDef *huart)
{
        UartBus *bus = Lookup(huart);
        if (bus != nullptr && bus->started_) bus->OnError();
}

// FreeRTOS 接收任务入口
void UartBus::RxTaskEntry(void *context) { static_cast<UartBus *>(context)->RxTask(); }
// FreeRTOS 发送任务入口
void UartBus::TxTaskEntry(void *context) { static_cast<UartBus *>(context)->TxTask(); }

// 启动接收：Normal 模式下一次收满或空闲都会结束，事件里再武装
HAL_StatusTypeDef UartBus::ArmRx()
{
        if (HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_dma_buffer_.data(), rx_dma_buffer_.size()) !=
            HAL_OK)
                return HAL_ERROR;
        // 只要 IDLE/TC：半缓冲的中断用不上
        __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
        return HAL_OK;
}

// ISR：Normal 模式下 HAL 传来的 size 就是本次新收到的字节数
void UartBus::OnRxEvent(uint16_t size)
{
        BaseType_t task_woken = pdFALSE;
        if (size != 0U)
        {
                const size_t sent =
                                xStreamBufferSendFromISR(rx_stream_buffer_, rx_dma_buffer_.data(), size, &task_woken);
                // StreamBuffer 满时新字节被丢弃：借错误回调让上层清残帧后重同步
                if (sent != size) pending_error_ = HAL_UART_ERROR_ORE;
        }
        // HAL 已把接收状态复位，这里立刻重新武装，避免漏掉紧随其后的数据
        if (ArmRx() != HAL_OK) rearm_pending_ = true;
        portYIELD_FROM_ISR(task_woken);
}

// ISR：通知 TX 任务本帧发完
void UartBus::OnTxComplete()
{
        BaseType_t task_woken = pdFALSE;
        (void) xSemaphoreGiveFromISR(tx_complete_semaphore_, &task_woken);
        portYIELD_FROM_ISR(task_woken);
}

// ISR：记下错误码并请求重新武装
void UartBus::OnError()
{
        pending_error_ = huart_->ErrorCode;
        rearm_pending_ = true;
}

// RX 任务：把字节递交给注册回调；超时醒来时补做重新武装与错误上报
void UartBus::RxTask()
{
        std::array<uint8_t, 64> data{};
        for (;;)
        {
                const size_t size =
                                xStreamBufferReceive(rx_stream_buffer_, data.data(), data.size(), kRxPollTicks);
                if (size != 0U && rx_callback_ != nullptr) rx_callback_(context_, data.data(), size);

                if (rearm_pending_)
                {
                        rearm_pending_ = false;
                        if (ArmRx() != HAL_OK)
                        {
                                // 错误可能把 huart 留在非 Ready 状态：中止一次再重开
                                (void) HAL_UART_AbortReceive(huart_);
                                if (ArmRx() != HAL_OK)
                                {
                                        rearm_pending_ = true;
                                        vTaskDelay(1U);
                                }
                        }
                }

                taskENTER_CRITICAL();
                const uint32_t error_code = pending_error_;
                pending_error_            = 0U;
                taskEXIT_CRITICAL();
                if (error_code != 0U) NotifyError(error_code);
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
                        NotifyError(huart_->ErrorCode);
                }
        }
}

// 在任务上下文向注册者通报错误码
void UartBus::NotifyError(uint32_t error_code)
{
        if (error_callback_ != nullptr) error_callback_(context_, error_code);
}
