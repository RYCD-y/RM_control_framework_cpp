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

// 一条 UART 总线：
//   RX：Normal 模式 DMA + IDLE/TC 事件 → StreamBuffer → RX 任务分发
//   TX：整帧入队 MessageBuffer → TX 任务 DMA 发送并等完成
// 每个接收事件结束后立刻重新武装 DMA，空窗只有一个 ISR 的时间。
class UartBus
{
        public:
                static constexpr size_t      kRxDmaSize     = 128U;
                static constexpr size_t      kRxStreamSize  = 512U;
                static constexpr size_t      kTxFrameSize   = 128U;
                static constexpr size_t      kTxQueueSize   = 1024U;
                static constexpr uint32_t    kTaskStackSize = 256U;
                static constexpr UBaseType_t kTaskPriority  = 4U;

                // 接收回调：data 只在回调执行期间有效
                using RxCallback    = void (*)(void *context, const uint8_t *data, size_t size);
                // 错误回调：在任务上下文调用
                using ErrorCallback = void (*)(void *context, uint32_t error_code);

                // 绑定 UART 句柄；构造不碰硬件也不占注册表
                explicit UartBus(UART_HandleTypeDef *huart);

                // 禁止拷贝：内部持有 FreeRTOS 对象与注册关系
                UartBus(const UartBus &) = delete;

                UartBus &operator=(const UartBus &) = delete;

                // 登记接收/错误回调：每条总线一份，须在 Start 之前；同参数重复登记视为成功
                HAL_StatusTypeDef RegisterCallback(RxCallback rx_callback, ErrorCallback error_callback,
                                                   void *     context);

                // 创建缓冲/任务并启动接收；要求 RX/TX DMA 均为 Normal 模式
                HAL_StatusTypeDef Start();

                // 整帧入队发送（不静默丢弃半帧，满则 Busy）
                HAL_StatusTypeDef Send(const uint8_t *data, size_t size);

                bool IsStarted() const { return started_; }

        private:
                static constexpr size_t     kMaxBuses       = 3U;
                // RX 任务的取数超时：只在异常时用来补上重新武装与上报错误
                static constexpr TickType_t kRxPollTicks    = pdMS_TO_TICKS(2U);
                // 单条报文的 DMA 发送完成等待上限
                static constexpr TickType_t kTxTimeoutTicks = pdMS_TO_TICKS(20U);

                // HAL 回调只有静态入口，靠注册表按句柄找回实例
                static bool Attach(UartBus *bus);

                // 按 UART 句柄找到已登记的总线实例
                static UartBus *Lookup(UART_HandleTypeDef *huart);

                // HAL 接收事件桥接
                static void RxEventThunk(UART_HandleTypeDef *huart, uint16_t size);

                // HAL 发送完成桥接
                static void TxCompleteThunk(UART_HandleTypeDef *huart);

                // HAL 错误桥接
                static void ErrorThunk(UART_HandleTypeDef *huart);

                // FreeRTOS 任务入口
                static void RxTaskEntry(void *context);

                // FreeRTOS 任务入口
                static void TxTaskEntry(void *context);

                // 启动或重启接收；任务与 ISR 都可调用，失败置 rearm_pending_ 交给任务重试
                HAL_StatusTypeDef ArmRx();

                // ISR：把本次收到的字节写进 StreamBuffer，然后重新武装
                void OnRxEvent(uint16_t size);

                // ISR：释放发送完成信号量
                void OnTxComplete();

                // ISR：记下错误码并请求重新武装
                void OnError();

                void RxTask();

                void TxTask();

                // 启动失败回滚：注销 HAL 回调、删除已建任务；可重复调用
                HAL_StatusTypeDef RollbackStart();

                // 在任务上下文通报错误
                void NotifyError(uint32_t error_code);

                UART_HandleTypeDef *huart_          = nullptr;
                RxCallback          rx_callback_    = nullptr;
                ErrorCallback       error_callback_ = nullptr;
                void *              context_        = nullptr;

                std::array<uint8_t, kRxDmaSize>         rx_dma_buffer_{};
                std::array<uint8_t, kTxFrameSize>       tx_dma_buffer_{};
                std::array<uint8_t, kTxQueueSize>       tx_storage_{};
                std::array<uint8_t, kRxStreamSize>      rx_stream_storage_{};
                StaticMessageBuffer_t                   tx_message_buffer_storage_{};
                StaticStreamBuffer_t                    rx_stream_buffer_storage_{};
                StaticSemaphore_t                       tx_complete_storage_{};
                StaticSemaphore_t                       tx_mutex_storage_{};
                StaticTask_t                            rx_task_storage_{};
                StaticTask_t                            tx_task_storage_{};
                std::array<StackType_t, kTaskStackSize> rx_task_stack_{};
                std::array<StackType_t, kTaskStackSize> tx_task_stack_{};
                MessageBufferHandle_t                   tx_message_buffer_     = nullptr;
                StreamBufferHandle_t                    rx_stream_buffer_      = nullptr;
                // 以下两个信号量必须保留，不能改成任务通知：
                //  1) StreamBuffer/MessageBuffer 内部用任务通知等待，FreeRTOS V10.3.1 只有一个
                //     通知槽且无 indexed API，再从 ISR 发通知会与其争用。
                //  2) 互斥量还需提供优先级继承，通知没有。
                SemaphoreHandle_t                       tx_complete_semaphore_ = nullptr;
                SemaphoreHandle_t                       tx_mutex_              = nullptr;
                TaskHandle_t                            rx_task_               = nullptr;
                TaskHandle_t                            tx_task_               = nullptr;
                // ISR 置位、RX 任务清零：接收需要重新武装
                volatile bool                           rearm_pending_         = false;
                // ISR 写入、RX 任务取出：待通报的错误码
                volatile uint32_t                       pending_error_         = 0U;
                bool                                    started_               = false;

                inline static std::array<UartBus *, kMaxBuses> buses_{};
};

#endif // THREE_FRICTION_HAL_UART_HPP
