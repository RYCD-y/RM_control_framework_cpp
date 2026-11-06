#ifndef THREE_FRICTION_HAL_UART_HPP
#define THREE_FRICTION_HAL_UART_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "main.h"
#include "message_buffer.h"
#include "semphr.h"
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

    // 回调签名：数据只在回调调用期间有效
    using RxCallback = void (*)(void* context, const uint8_t* data, size_t size);
    // 错误回调签名
    using ErrorCallback = void (*)(void* context, uint32_t error_code);

    // 绑定 UART 句柄并登记；构造不碰硬件
    explicit UartBus(UART_HandleTypeDef* huart);

    // 仅未启动时可析构，用于从注册表移除
    ~UartBus();

    // 禁止拷贝：内部持有 FreeRTOS 对象与注册关系
    UartBus(const UartBus&) = delete;

    UartBus& operator=(const UartBus&) = delete;

    // 登记接收/错误回调（需在 Start 之前）
    HAL_StatusTypeDef RegisterCallback(RxCallback rx_callback, ErrorCallback error_callback,
                                       void* context);
    // 注销回调（仅限未启动时）
    HAL_StatusTypeDef UnregisterCallback(void* context);
    // 创建缓冲/任务并启动循环 DMA 接收
    HAL_StatusTypeDef Start();
    // 整帧入队发送（不静默丢弃半帧，满则 Busy）
    HAL_StatusTypeDef Send(const uint8_t* data, size_t size);

    // 只读状态与计数：已启动/丢字节/发送错误/接收错误
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

    // 根据 UART 句柄找到对应总线实例
    static UartBus* Find(UART_HandleTypeDef* huart);
    // HAL 回调桥接：只做中继，不解析协议
    static void RxEventThunk(UART_HandleTypeDef* huart, uint16_t size);
    // HAL 发送完成桥接
    static void TxCompleteThunk(UART_HandleTypeDef* huart);
    // HAL 错误桥接
    static void ErrorThunk(UART_HandleTypeDef* huart);
    // FreeRTOS 任务入口
    static void RxTaskEntry(void* context);
    // FreeRTOS 任务入口
    static void TxTaskEntry(void* context);

    // ISR：按 DMA 指针增量把新字节搬进 StreamBuffer
    void OnRxEvent(uint16_t size);
    // ISR：释放发送完成信号量
    void OnTxComplete();
    // ISR：标记接收需重启；由 RX 任务在超时轮询中处理
    void OnError();
    // RX 任务：分发给注册回调，并处理重启
    void RxTask();
    // TX 任务：逐条取出报文用 DMA 发送并等完成
    void TxTask();

    // 中止旧传输 → 清残帧 → 重启接收
    HAL_StatusTypeDef RestartRx();

    // 启动失败时的回滚：注销 HAL 回调、删除已建任务；可重复调用
    HAL_StatusTypeDef RollbackStart();

    // 在任务上下文通报错误
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
    // 以下两个信号量必须保留，不能改成任务通知：
    //  1) StreamBuffer/MessageBuffer 内部用任务通知等待，FreeRTOS V10.3.1 只有一个
    //     通知槽且无 indexed API，再从 ISR 发通知会与其争用。
    //  2) 互斥量还需提供优先级继承，通知没有。
    SemaphoreHandle_t tx_complete_semaphore_ = nullptr;
    SemaphoreHandle_t tx_mutex_ = nullptr;
    TaskHandle_t rx_task_ = nullptr;
    TaskHandle_t tx_task_ = nullptr;
    volatile uint32_t rx_produced_position_ = 0U;
    volatile uint32_t rx_dropped_bytes_ = 0U;
    volatile uint32_t tx_errors_ = 0U;
    volatile uint32_t rx_errors_ = 0U;
    volatile bool rx_restart_pending_ = false;  // ISR 置位，RX 任务轮询清零
    bool started_ = false;
};

#endif  // THREE_FRICTION_HAL_UART_HPP
