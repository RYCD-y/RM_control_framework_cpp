# 信号量 → 任务通知 评估与改造

## Context

用户询问：项目中的信号量能否换成直接任务通知。

需要先厘清：**本项目所有信号量都集中在 `UartBus`（`hal_uart`）**，共 2 个；其余模块（CAN、电机、LED、蜂鸣器）没有使用信号量。因此问题实际是"UartBus 的两个同步对象能否替换"。

另外发现一处**已存在的混用隐患**（RX 任务同时使用 StreamBuffer 与任务通知），值得一并修正。

## 关键约束（已核实）

1. FreeRTOS 版本 **V10.3.1**（`Middlewares/Third_Party/FreeRTOS/Source/include/task.h:46`）。
2. V10.3.1 **没有 indexed 通知 API**：`ulTaskNotifyTakeIndexed` / `vTaskNotifyGiveIndexedFromISR` 在 `task.h` 中出现次数为 0，也不支持 `configTASK_NOTIFICATION_ARRAY_ENTRIES`（10.4.0 才引入）。每个任务**只有一个通知槽**。
3. `stream_buffer.c:565` 与 `stream_buffer.c:771` 显示 `xStreamBufferReceive` 内部用 `xTaskNotifyWait(0, 0, NULL, xTicksToWait)` 阻塞 —— **StreamBuffer / MessageBuffer 本身占用该任务的通知槽**。
4. `configUSE_MUTEXES = 1`（`Inc/FreeRTOSConfig.h:70`），`configUSE_TASK_NOTIFICATIONS` 未显式定义（默认为 1）。

## 现状清单

| 对象 | 位置 | 当前用途 |
|---|---|---|
| `tx_complete_semaphore_` | `hal_uart.cpp:99` 创建；`:236` ISR give；`:290-295` TX 任务 take | 等待 DMA 发送完成 |
| `tx_mutex_` | `hal_uart.cpp:100` 创建；`:156-158` `Send()` 加锁 | 保护 MessageBuffer 单写者约束、支持多任务发送 |
| （非信号量）`rx_restart_pending_` + `vTaskNotifyGiveFromISR` | `:227`、`:246`、`:266` | 唤醒 RX 任务处理接收重启 |

## 结论：两个信号量都不建议替换

### `tx_mutex_` —— 不可替换

- 它保护 `xMessageBufferSend` 的**单写者**约束（FreeRTOS 明确要求 StreamBuffer/MessageBuffer 只有一个写者）。
- 互斥量提供**优先级继承**，任务通知没有。
- 用通知实现自旋锁会引入忙等与优先级反转风险，属于倒退。

### `tx_complete_semaphore_` —— 不可替换

TX 任务长期阻塞在 `xMessageBufferReceive(..., portMAX_DELAY)`，该调用内部占用任务通知槽。若同时把 DMA 完成改为 `vTaskNotifyGiveFromISR(tx_task_)` + `ulTaskNotifyTake`，二者会**争用同一个通知值**：

- MessageBuffer 数据到达时的通知可能被 `ulTaskNotifyTake` 取走 → MessageBuffer 误判为无数据；
- DMA 完成通知可能被 `xTaskNotifyWait` 消费 → TX 任务白等到 20 ms 超时后才中止；
- 结果是随机假唤醒、偶发超时与"发送完成丢失"。

这正是 FreeRTOS 官方对 StreamBuffer 的告警场景。**唯有**升级到 ≥10.4 使用 indexed notification（TX 用 index 1）或改用 EventGroup 才能避免，两者都超出"简化"的目的。

## 建议方案（净收益、零风险）

保留 `tx_mutex_` 与 `tx_complete_semaphore_`，**只修 RX 路径的混用**：

1. 删除 RX 侧的 `vTaskNotifyGiveFromISR`（`hal_uart.cpp:227`、`:246`）与 `ulTaskNotifyTake`（`:266`）。
2. ISR 只置位 `volatile bool rx_restart_pending_`；RX 任务依赖已有的 `pdMS_TO_TICKS(2U)` 超时轮询检测并处理。
3. 顺带消除 `xStreamBufferReset` 与 ISR 并发写的竞态：重启流程改为「先置位标志 → 任务内 `HAL_UART_AbortReceive` 停 DMA → reset 缓冲 → 重新开启接收」。
4. 净效果：少两处 ISR 通知调用、消除一处真实竞态；信号量数量不变（仍为 2），因为它们在当前 FreeRTOS 版本下是**正确**选择。

## 若不接受上述结论

需要用户明确选择其一（均超出"简化"范畴）：

- **A. 升级 FreeRTOS 到 ≥10.4**，TX 改用 `vTaskNotifyGiveIndexedFromISR(tx_task_, 1)` / `ulTaskNotifyTakeIndexed(1, ...)`，可安全去掉 `tx_complete_semaphore_`。代价：替换内核源码，回归面大。
- **B. 改用 EventGroup** 承载 DMA 完成（不占通知槽）。代价：多一个对象，代码并不更简单。
- **C. 重构 TX 为轮询 DMA 状态**，彻底去掉同步对象。代价：忙等或固定延时，实时性变差。

## Files to modify

- `UserCode/HAL/hal_uart.cpp`：移除 RX 侧通知调用，重排重启流程。
- `UserCode/HAL/hal_uart.hpp`：如需要，更新 `rx_restart_pending_` 注释。
- （仅方案 A 才涉及）`Middlewares/Third_Party/FreeRTOS/**`、`Inc/FreeRTOSConfig.h`。

## Reuse

- 现有 `UartBus::RestartRx()`、`RxDroppedBytes()`、`RxErrors()` 计数保持不变，仅调整触发方式。
- `xStreamBufferReset` / `HAL_UART_AbortReceive` 已存在，无需新增 API。

## Steps

- [ ] 移除 `OnRxEvent` 与 `OnError` 中的 `vTaskNotifyGiveFromISR`。
- [ ] 移除 `RxTask` 中的 `ulTaskNotifyTake`，改为仅检查 `rx_restart_pending_`。
- [ ] 重启流程改为「AbortReceive → StreamBufferReset → 重新 Receive」，避免与 ISR 写入竞态。
- [ ] 保持 `tx_mutex_` 与 `tx_complete_semaphore_` 不变，并加注释说明为何不能换成通知。

## Verification

- `cmake --build --preset Debug` / `Release`：0 错误 0 警告。
- `ctest --test-dir tests/host/build`：7/7 通过。
- 静态检查：`clangd --check` 对 `hal_uart.cpp` 无代码诊断。
- 上板（未做则不得声称通过）：串口持续高速收流 + 主动注入错误（如短接/波特率不匹配），确认重启后仍能恢复收包、无死锁、`RxErrors` 递增而数据不丢帧。

## 待确认

1. 是否接受"保留 2 个信号量、只修 RX 混用"的方案？
2. 还是要求彻底去掉同步对象（走 A/B/C，需升级内核或接受实时性下降）？
