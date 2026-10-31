# CAN 回调接口 + 通用回调处理（参考 control_framework_v2）

> 已确认：回调形式 = **B（C++ 模板，可绑成员函数/lambda）**；范围 = **③（接收分发 + 错误/掉线 + 总线重启 + 发送队列）**；`BusNode::onFrame` **保留**；**支持多播**（同一 ID 多订阅者）。

## Context

`three_friction` 现在的 CAN 接收链路是"硬编码"的：

- `UserCode/DRV/motor/drv_dji_motor.cpp` 里用匿名 namespace 维护 `s_nodes[]` / `registerNode()`，
  并在该文件里定义 `extern "C" void HAL_CAN_RxFifo0MsgPendingCallback()`。
- **通用分发逻辑写在了电机驱动里**：任何新 CAN 设备（超级电容 CM01、裁判系统、自定义从机）想收帧，
  都要再抄一份注册表 + 一份 HAL 回调。
- HAL 层没有"注册接收回调"的接口，没有总线错误回调、没有掉线判定、没有发送队列——
  `HAL_CAN_AddTxMessage` 在业务任务里直接调，邮箱满时丢帧是静默的。

参考工程 `control_framework_v2` 的做法（`UserCode/HAL/hal_can.c`）：

```
CAN_Node_Register()                  静态表 + 去重            (hal_can.c:52)
HAL_CAN_RxFifo0MsgPendingCallback()  按 (hcan, Feedback_ID & Mask) 匹配 → handler(节点)  (hal_can.c:95)
MessageBuffer + CAN_TxTask           中断只入队，任务里等邮箱空位再发  (hal_can.c:120/222)
Err_HandleTypeDef                    每节点心跳，SRV 层 10ms 周期遍历    (Common/err.c, SRV/srv_error_monitor.c)
CAN_Restart()                        Stop→DeInit→Init→Filter→Start    (hal_can.c:34)
```

目标：把这套「注册表 + 分发 + 回调 + 发送队列 + 错误处理」下沉到 HAL 层，
DRV 只负责"挂回调 + 解包"。

## 现状代码地图

| 文件 | 相关内容 |
|---|---|
| `UserCode/HAL/hal_bus.hpp` | `BusNode` 抽象类：`virtual send()` / `virtual onFrame(const uint8_t*, uint16_t)`、`id_/feedback_id_/id_mask_` |
| `UserCode/HAL/hal_can.hpp` | `canNode : BusNode`；`can_cfg`（`kCan2FilterBank`/`kIdMaskNone`/`kIdMaskLow8`/`kStdIdMask`/`kExtIdMask`）；`send/sendEx/filterInit/getHcan` |
| `UserCode/HAL/hal_can.cpp` | `send / sendEx / filterInit` |
| `UserCode/DRV/motor/drv_dji_motor.cpp` | `s_nodes[]` + `registerNode()` + `HAL_CAN_RxFifo0MsgPendingCallback`（**要搬走**）；`djiMotor::onFrame(data,len)`（保留） |
| `UserCode/ENT/ent_main.cpp` | `filterInit → HAL_CAN_Start` 启动顺序 |
| `UserCode/Common/utils.hpp` | `utils::be_i16 / put_be_i16`（可复用） |

FreeRTOS 约束（`Inc/FreeRTOSConfig.h`）：

| 项 | 值 | 影响 |
|---|---|---|
| `configTOTAL_HEAP_SIZE` | 24576 B | TxTask 栈 1 KB + 队列 ~0.5 KB，够用 |
| `configMAX_PRIORITIES` | 7（0~6） | TxTask 用 **6**，高于 `debugTask`(5) |
| `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | 5 | CAN NVIC 优先级正好 5 → **可以在 ISR 里调 `FromISR` API**（不能再调低数值） |
| `configTICK_RATE_HZ` | 1000 | 1 tick = 1 ms，掉线判定用 tick 计数即可 |

## Approach

### 1. 通用帧结构 `canFrame`（hal_can.hpp）

脱离 HAL 的 `CAN_TxHeaderTypeDef`，收发共用，20 字节：

```cpp
struct canFrame
{
        CAN_HandleTypeDef *hcan     = nullptr;
        uint32_t           id       = 0;       // StdId(11b) 或 ExtId(29b)
        uint8_t            data[8]  = {};
        uint8_t            dlc      = 8;
        bool               extended = false;
        bool               rtr      = false;
};
```

### 2. 回调接口（`canNode`，hal_can.hpp）

回调统一为 **函数指针 + 上下文**（零分配、可 ISR 调用），模板只做"生成 thunk"：

```cpp
using canRxCallback = void (*)(void *ctx, const canFrame &frame);

// 底层
void attachRx(canRxCallback fn, void *ctx = nullptr);
void detachRx();

// 派生类成员函数（推荐写法）
template <typename T, void (T::*M)(const canFrame &)>
void attachRxFrom(T *self)
{
        attachRx(&thunkFrom<T, M>, self);
}

// 本类成员函数
template <void (canNode::*M)(const canFrame &)>
void attachRxSelf()
{
        attachRx(&thunkSelf<M>, this);
}
```

两个 thunk 都是 `static` 模板，编译期实例化，无运行期开销：

```cpp
template <typename T, void (T::*M)(const canFrame &)>
static void thunkFrom(void *ctx, const canFrame &f) { (static_cast<T *>(ctx)->*M)(f); }
```

无捕获 lambda 也能直接传（可隐式转函数指针）：`node.attachRx([](void *, const canFrame &f){...});`

**虚函数保留**（用户要求），并加一层 CAN 专用虚接口，默认桥接旧的 `onFrame`：

```cpp
// BusNode 不变：virtual HAL_StatusTypeDef onFrame(const uint8_t*, uint16_t) = 0;

class canNode : public BusNode
{
        public:
                // 有 attachRx 回调时走回调；否则走这里
                virtual void onCanFrame(const canFrame &frame)
                {
                        (void) onFrame(frame.data, frame.dlc);   // 桥接旧接口，djiMotor 不改也能工作
                }
};
```

分发的**优先级**：`attachRx` 回调 > `onCanFrame` 虚函数。
`djiMotor` 改成 override `onCanFrame`（能拿到 ID/IDE，为将来扩展帧预留），`onFrame` 保留但不再实现业务。

### 3. 通用分发路由（hal_can.cpp，`namespace can_rx`）

表项独立于节点，天然支持**多播**：

```cpp
namespace can_rx
{
        inline constexpr uint8_t kMaxSubscribers = 32;      // 与参考工程 CAN_NODE_MAX 一致

        struct entry
        {
                CAN_HandleTypeDef *hcan   = nullptr;
                uint32_t           id     = 0;
                uint32_t           mask   = 0;
                canRxCallback      fn     = nullptr;
                void              *ctx    = nullptr;
                canNode           *owner  = nullptr;   // 用于心跳/统计，可为空
        };

        bool subscribe(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t mask,
                       canRxCallback fn, void *ctx, canNode *owner = nullptr);
        bool unsubscribe(canRxCallback fn, void *ctx);
        uint8_t count();
        void dispatch(CAN_HandleTypeDef *hcan, const canFrame &frame);  // 中断上下文调用
}
```

`dispatch` 遍历全部表项，`(frame.id & mask) == (id & mask)` 全部命中都调用 → 多播。

`canNode::attachRx` 内部就是 `can_rx::subscribe(hcan_, feedback_id_, id_mask_, fn, ctx, this)`，
所以节点的心跳也在 `dispatch` 里顺手刷新。

### 4. HAL 接收回调（hal_can.cpp，从 drv_dji_motor.cpp 搬来）

```cpp
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
        CAN_RxHeaderTypeDef hdr;
        canFrame            f;

        while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)     // 一次取干净
        {
                if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, f.data) != HAL_OK) break;

                f.hcan     = hcan;
                f.id       = (hdr.IDE == CAN_ID_EXT) ? hdr.ExtId : hdr.StdId;
                f.extended = (hdr.IDE == CAN_ID_EXT);
                f.rtr      = (hdr.RTR == CAN_RTR_REMOTE);
                f.dlc      = static_cast<uint8_t>(hdr.DLC);

                can_rx::dispatch(hcan, f);
        }
}
```

比参考工程多一个改进：用 `while` 把 FIFO 取空（参考工程每次只取 1 帧，FIFO 里剩多帧时要靠下一次中断触发）。

### 5. 发送队列（hal_can.cpp，`namespace can_tx`）

```cpp
namespace can_tx
{
        inline constexpr uint8_t  kDepth      = 16;    // 16 帧 ≈ 1kHz 下 16ms 缓冲
        inline constexpr uint32_t kTimeoutMs  = 10;    // 邮箱全满时的等待上限
        inline constexpr uint32_t kTaskStack  = 256;   // words
        inline constexpr UBaseType_t kTaskPrio = 6;    // = configMAX_PRIORITIES-1

        bool start();                                  // 创建 MessageBuffer + TxTask
        bool enqueue(const canFrame &frame);            // 任意任务调用；未 start 时退化为直接发
        HAL_StatusTypeDef sendNow(const canFrame &frame);   // 直接发（绕过队列，ISR 可用）
        uint32_t dropCount();
}
```

- 队列用 `xMessageBufferCreate(kDepth * (sizeof(canFrame) + 4))` ≈ 16×24 = 384 B。
- `HAL_CAN_TxMailbox{0,1,2}CompleteCallback` → `vTaskNotifyGiveFromISR(TxTask)`（优先级 5，允许）。
- TxTask：`xMessageBufferReceive` → 等邮箱空位（task notification，超时 `kTimeoutMs`）→ `HAL_CAN_AddTxMessage` → 失败则 `drop_count++`。
- `canNode::send()/sendEx()` 改为 `can_tx::enqueue(...)`，返回值语义变为"入队是否成功"；
  需要同步确认发送的地方用 `can_tx::sendNow()`。

### 6. 错误 / 掉线 / 总线重启（hal_can.cpp，`namespace can_bus`）

```cpp
namespace can_bus
{
        inline constexpr uint32_t kOfflineTimeoutMs = 100;   // 参考工程 10ms 周期 × 10

        extern volatile uint32_t errorCode[2];   // [0]=CAN1 [1]=CAN2，HAL_CAN_ErrorCallback 写入
        uint32_t errorCodeOf(CAN_HandleTypeDef *hcan);
        HAL_StatusTypeDef restart(CAN_HandleTypeDef *hcan);   // Stop→DeInit→Init→filterInit→Start
}

// canNode 增加（掉线判定，供 SRV 周期轮询）
bool     isOnline() const;             // 距上次收到反馈超过 kOfflineTimeoutMs 判离线
void     setOfflineTimeoutMs(uint32_t ms);
uint32_t rxCount() const;
uint32_t txDropCount() const;
```

`dispatch` 命中时刷新 `last_rx_tick_` 与 `rx_count_`（需要在表项里带 `owner`）。
掉线策略（停机/报警）留给 SRV 层轮询，不在 HAL 里做决策——与参考工程 `srv_error_monitor.c` 的分层一致。

### 7. 启动顺序（ent_main.cpp）

```cpp
canNode::filterInit(&hcan1);   HAL_CAN_Start(&hcan1);
canNode::filterInit(&hcan2);   HAL_CAN_Start(&hcan2);
can_tx::start();               // 建队列 + TxTask（必须在任何 send 之前）
```

## Files to modify

| 文件 | 改动 |
|---|---|
| `UserCode/HAL/hal_can.hpp` | 新增 `canFrame`、`canRxCallback`、`canNode::attachRx/attachRxFrom/attachRxSelf/onCanFrame/isOnline/rxCount`、`namespace can_rx / can_tx / can_bus` 声明 |
| `UserCode/HAL/hal_can.cpp` | 路由表 + `dispatch` + `HAL_CAN_RxFifo0MsgPendingCallback` + `can_tx`（MessageBuffer/TxTask/邮箱回调）+ `HAL_CAN_ErrorCallback` + `restart` |
| `UserCode/HAL/hal_bus.hpp` | 不改（`onFrame` 保留；仅确认虚析构/纯虚现状） |
| `UserCode/DRV/motor/drv_dji_motor.hpp` | 构造里 `attachRxFrom<djiMotor, &djiMotor::onCanFrame>(this)`；新增 `void onCanFrame(const canFrame&) override` |
| `UserCode/DRV/motor/drv_dji_motor.cpp` | **删除**匿名 namespace 的 `s_nodes[]`/`registerNode()` 与 `HAL_CAN_RxFifo0MsgPendingCallback`；`onFrame` 改 `onCanFrame`；`groupInit` 去掉 `registerNode` |
| `UserCode/ENT/ent_main.cpp` | 加 `can_tx::start()` |
| `UserCode/APP/debug.cpp` | 示例：额外挂一个无捕获 lambda 做 CAN 日志（演示多播 + B 形式回调） |

## Reuse

| 复用什么 | 位置 |
|---|---|
| `can_cfg::kIdMaskNone / kIdMaskLow8 / kCan2FilterBank` | `UserCode/HAL/hal_can.hpp`（现成常量，不新造） |
| `canNode::filterInit()` / `getHcan()` / `send()` / `sendEx()` | `UserCode/HAL/hal_can.cpp`（扩展而非重写） |
| `utils::be_i16 / put_be_i16` | `UserCode/Common/utils.hpp`（`djiMotor` 解包继续用） |
| `djiMotor::groupSend()` 的槽位/大端打包 | `UserCode/DRV/motor/drv_dji_motor.cpp`（不改，只把发送换成队列） |
| 静态注册表 + 去重 + 遍历匹配 | `control_framework_v2/UserCode/HAL/hal_can.c:52-108` |
| MessageBuffer + TxTask + task notification 等邮箱 | `control_framework_v2/UserCode/HAL/hal_can.c:120-231` |
| `CAN_Restart()` 的 Stop→DeInit→Init→Filter→Start 顺序 | `control_framework_v2/UserCode/HAL/hal_can.c:34-46` |
| 心跳/掉线"机制在 HAL、策略在 SRV"的分层 | `control_framework_v2/UserCode/Common/err.h` + `UserCode/SRV/srv_error_monitor.c` |

## Steps

- [ ] 1. `hal_can.hpp`：加 `canFrame`、`canRxCallback`、两个 thunk 模板、`canNode::attachRx*` / `detachRx` / `onCanFrame`（默认桥接 `onFrame`）
- [ ] 2. `hal_can.hpp`：加 `can_rx`（`subscribe/unsubscribe/dispatch/count`）、`can_tx`（`start/enqueue/sendNow/dropCount`）、`can_bus`（`errorCode/restart`）声明
- [ ] 3. `hal_can.cpp`：实现 `can_rx` 表（32 项、去重、多播匹配、刷新心跳）
- [ ] 4. `hal_can.cpp`：把 `HAL_CAN_RxFifo0MsgPendingCallback` 从 `drv_dji_motor.cpp` 搬来，改为 `while` 取空 FIFO + 组 `canFrame` + `dispatch`
- [ ] 5. `hal_can.cpp`：实现 `can_tx`（`xMessageBufferCreate` + TxTask + 3 个邮箱完成回调 + 超时丢帧计数）；`canNode::send/sendEx` 改走 `enqueue`
- [ ] 6. `hal_can.cpp`：`HAL_CAN_ErrorCallback` 记录 `errorCode[2]`；实现 `can_bus::restart()`
- [ ] 7. `djiMotor`：`onFrame(data,len)` → `onCanFrame(const canFrame&)`；构造里 `attachRxFrom`；删本地注册表/回调
- [ ] 8. `ent_main.cpp`：`can_tx::start()`；`debug.cpp`：加一个 lambda 订阅示例（多播演示）
- [ ] 9. 删掉 `drv_dji_motor.cpp` 里遗留的 `registerNode` 调用与表

## Verification

- [ ] `ninja -C build/Debug` / `ninja -C build/Release`：0 error / 0 warning
- [ ] `arm-none-eabi-nm build/Debug/three_friction.elf | grep HAL_CAN_RxFifo0MsgPendingCallback`：**只有一个 `T` 定义**
- [ ] `arm-none-eabi-size`：Flash 增量 < 2 KB，FreeRTOS heap 余量 > 4 KB（可用 `xPortGetFreeHeapSize()` 打印）
- [ ] host 侧单测路由匹配：把 `can_rx` 的匹配逻辑抽成纯函数（不依赖 HAL），用 `g++` 跑
      「ID 0x201/0x202、mask 0x7FF、extended 帧、多播场景」的用例
- [ ] 板端（若有硬件）：三台 M3508 反馈后 `getSpeed()/getAngle()` 正常刷新；
      拔掉 CAN 线 → `isOnline()` 100 ms 内变 false；`txDropCount()` 在总线断开时增长
- [ ] 复查：`debug.cpp` 里 m1/m2/m3 仍能用 `kC620_M3508` 初始化并 `groupSend()`
