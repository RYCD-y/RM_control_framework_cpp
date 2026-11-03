# 电机驱动与 UART 层迁移计划

## Context

- 为 `three_friction` 加入达妙、ZDT Emm V5 和无反馈舵机；电机协议参考 `../control_framework_v2` 与 `/home/ye/Documents/RM/RM手册`，舵机设计按用户指定参考 `../2026gcxl`。
- 使用 C++17、面向对象设计和 Google C++ 命名；保持实现小、注释少。
- 最新范围：UART 使用 FreeRTOS 流/消息缓冲与 DMA 收发，支持回调注册；CAN 与现有代码允许必要重构，最终手写代码风格统一。
- 舵机品牌为 YOUMI，驱动设计依据用户补充指定的 `../2026gcxl/UserCode/BSP/bsp_servo.*` 与 `bsp_tim.*`；不再以淘宝页面可访问性作为实施阻塞。
- 当前处于规划阶段，仅修改本计划，批准前不修改源码或运行构建。
- 工作区已有用户改动：`UserCode/APP/debug.cpp` 和两个 build 日志文件，实施时保留。

## Approach

- 按扭矩、速度、位置拆分无状态纯接口，最多一层具体实现；反馈与控制分离，无反馈舵机不提供实际位置。公共物理单位采用 rad、rad/s、N·m，名称显式包含单位；原有 DJI degree/rpm 在迁移时同步全部调用，不静默变更含义。
- 不引入万能 `MotorBase`、手写 vtable、RTTI 能力查询、工厂或通用 Bus 框架。共同控制接口采用 `SetTarget...()` 保存目标、`Send()` 显式提交，返回接受/入队状态而不是物理到位；设备专有命令保留具体类型接口。
- 达妙首版以 MIT 为共同能力基线：`DmMotor` 提供扭矩接口、完整 `MitCommand`（位置/速度/kp/kd/前馈力矩）、使能/失能/清错/零点与反馈。MIT 量程显式配置并与实际固件匹配；原生位置速度/速度保留显式模式校验的协议 API，本期不额外创建一套模式子类。纯扭矩仅在 MIT 模式下以 kp=kd=0 实现，其他模式返回不匹配，调用方不伪造能力。JH11 专有 CSP/CSV/CST、在线改参和持久化写入不在默认范围。
- ZDT 实现速度/位置接口与独立反馈；固定 `0x6B` 校验为本期明确支持方式，支持使能/失能/停止/零点/速度/绝对与相对位置/位置速度状态查询。在同一驱动文件内使用小型 `ZdtBus` 管理每 UART 的地址表、帧解析和单在途命令（包含控制应答与查询），忙时返回 Busy，超时恢复；不再额外构建通用事务队列。
- ZDT 按功能码长度累积/校验/重同步，注册同地址跨 UART 合法、同 UART 重复地址拒绝；查询由调用方显式触发，不增加轮询任务。切换目标模式不重发旧目标，只有成功接受命令才更新提交状态，协议应答与物理到位状态分别保存。
- 舵机参考 `2026gcxl`，移植为仅实现位置接口的 C++ `PwmServo`，构造注入定时器/通道与标定，保留角度/脉宽设置、反向、限幅和 PWM 启停；仅报告目标，不虚构在线/到位/实际位置。
- 提供与参考工程一致、明确命名为参考预设的 0–π rad（0–180°）、500–2500 µs、50 Hz 配置，不宣称它已获 YOUMI 厂家验证；支持覆盖标定，不自动绑定硬件或在主入口启动。`Init()` 只校验和准备，`Enable()` 显式启动，并在 PWM Start 前装载有效 CCR，避免先输出旧值/零值。
- 复用参考 `bsp_tim.*` 的 APB1/APB2 定时器时钟与预分频换算思路，以实际寄存器 PSC/ARR 校验 PWM 周期、最大脉宽、有限输入和通道支持；不因接入舵机修改同一 TIM 的其他通道频率。
- 保留现有 DJI 功能，允许修复 CAN 正确性问题与统一 API 命名，不引入无关抽象。
- 每个 UART 句柄只对应一个 `UartBus`，显式 `Init()`/`Start()`，构造不访问硬件，禁止拷贝。HAL 层注册 RxEvent/TxComplete/Error 静态桥接回调，上层注册函数指针与 `void* context`，不使用 `std::function`。
- RX：循环 DMA + IDLE/HT/TC 按新增偏移写入一个 FreeRTOS StreamBuffer，处理回卷和重复事件；UART 与 RX DMA 中断使用同一合法优先级避免生产者重入。RX 任务读取字节并调用注册接收处理器；不把一次 IDLE 当成完整协议帧。
- TX：FreeRTOS MessageBuffer 保存完整报文，多任务发送通过互斥保护；入队非阻塞、失败返回 Busy，不默默丢半帧。单 TX 任务将报文复制至持久 DMA 暂存区，通过独立二值信号量等待完成，不能复用 StreamBuffer/MessageBuffer 使用的任务通知。
- 每个启用总线使用一对 RX/TX 任务与固定容量内存，优先 FreeRTOS 静态创建接口；DMA 暂存区位于 DMA 可访问 SRAM，禁止放入 CCM。容量、任务栈和优先级集中配置，不另写一套环形缓冲。
- 回调在 RX/TX 工作任务中执行，ISR 仅复制字节、计数、释放信号量；注册使用固定容量表，拒绝重复或超容量，回调数据仅在调用期间有效。注册/注销限定总线停止时进行，引用对象必须比启动的总线活得更久；回调不等待同一总线、不在中断中注册或注销。
- 记录 RX 丢字节、TX 超时/错误和重启失败；丢字节通知协议层清理残帧后重同步，TX 超时中止当前 DMA、隔离迟到完成信号再处理下条消息，不能让任务永久等待。初始化任一步失败须回滚已注册回调和已创建资源；启用总线的任务、缓冲与信号量不在运行期销毁。
- 达妙按手册核对，ZDT 使用 Emm V5；YOUMI 按用户指定的 `2026gcxl` PWM 舵机设计移入，参数可配置，实机端点仍需低风险校准。
- 统一 `UserCode/` 和主机测试的 Google C++ 命名：类型 `UpperCamelCase`，普通函数 `UpperCamelCase()`，变量/命名空间 `snake_case`，类成员尾随 `_`，常量 `kUpperCamelCase`，头源文件 `snake_case.hpp/.cpp`；小型访问器按一致的 Google 属性访问风格。使用 `<cstdint>`、`std::array`、`enum class`、`explicit`、`override`、`nullptr`，以项目 namespace 管理自有代码。
- 不批量重命名 HAL、FreeRTOS、ETL、CubeMX 生成代码及必须保留的 C ABI 入口/回调符号；这些是统一风格的明确边界，不是改坏外部 API。成熟控制/滤波算法只做机械命名和格式统一，数值问题如与本次无关则单独记录。

## Files to modify

实施范围（同目录新文件采用现有 `.hpp/.cpp` 扩展名）：

- `UserCode/DRV/motor/motor_base.hpp`：控制能力接口与现有兼容。
- `UserCode/DRV/motor/drv_dm_motor.hpp/.cpp`：达妙驱动。
- `UserCode/DRV/motor/drv_zdt_motor.hpp/.cpp`：`ZdtMotor`、小型 `ZdtBus` 与 Emm V5 帧解析，不复制整份旧 `Emm_V5.c`。
- `UserCode/DRV/motor/drv_servo.hpp/.cpp`：`PwmServo`，参考预设与自定义标定，无反馈位置接口。
- `UserCode/HAL/hal_tim.hpp/.cpp`：仅提取必要的定时器时钟/tick 换算小函数，供 PWM 舵机使用；若已有等效实现则直接复用，不另造复杂定时器类。
- `UserCode/HAL/hal_uart.hpp/.cpp`：`UartBus`、DMA 收发、FreeRTOS 缓冲与回调注册。
- `UserCode/HAL/hal_can.hpp/.cpp`、`UserCode/DRV/motor/drv_dji_motor.hpp/.cpp`：命名统一、协议/分组边界修复。
- `UserCode/Common/vofa.hpp/.cpp`、`UserCode/ENT/ent_main.cpp`、`ent_devices.hpp`：VOFA 通过统一 UART 通道入队，不再单独占用 HAL 回调。
- `UserCode/Common/`、`UserCode/DRV/{led,buzzer}/`、`UserCode/SRV/`、`UserCode/APP/`、`UserCode/ENT/`：逐文件统一手写 C++ API、成员、常量和对应调用点；不重写成熟算法。
- `.clang-format`：Google 基础风格，仅格式化自有 C++ 代码。`tests/host/CMakeLists.txt` 与 `tests/host/{cases,stubs}/`：构建修复、HAL/RTOS 桩与协议测试。
- `Src/usart.c`、`Src/dma.c`、`Src/stm32f4xx_it.c`、`Inc/stm32f4xx_it.h`、`three_friction.ioc`：同步需要的 RX circular、USART1 IRQ 与相关 RTOS-safe NVIC 配置；保留 CubeMX 命名。
- `CMakeLists.txt`/工具链与链接脚本仅在实际需要时调整；不覆盖用户调试逻辑，用户文件中的 API 改名随调用点同步。

## Reuse

- `UserCode/HAL/hal_can.hpp/.cpp`：现有 `CanNode` 注册、CAN 收发与帧回调。允许统一命名并修复长度静默截断、帧类型/远程帧校验等问题；不照搬参考工程的完整 Bus 框架。
- `UserCode/DRV/motor/drv_dji_motor.hpp`：已有参数配置与分组发送。
- `CMakeLists.txt`：C++17 与 `UserCode/**/*.cpp` 自动收集，不预设需要改构建源列表。
- `Src/usart.c`：已有 USART1/6 115200 8N1 与 USART3 100000 偶校验配置，不能未经确认占用或改参数。
- 参考工程 `UserCode/DRV/motor/drv_motor_dm.*`、`drv_motor_zdt_emm_V5.*`、`Emm_V5.*` 与 `UserCode/HAL/hal_uart.*`：复用帧结构和接入思路，改写成 C++，不照搬完整 C 框架。
- 参考 UART 的双任务/ReceiveToIdle DMA 设计可参考，但 TX 字节流会丢失消息边界，且没有保护多写者；新实现 TX 使用完整消息缓冲，DMA 完成同步不与 StreamBuffer 内部任务通知混用。FreeRTOS 官方 V10 参考手册与 Intertask coordination 文档确认单写者/单读者和通知占用约束。
  - https://www.freertos.org/media/2018/FreeRTOS_Reference_Manual_V10.0.0.pdf
  - https://docs.aws.amazon.com/freertos/latest/userguide/inter-task-coordination.html
- `Inc/FreeRTOSConfig.h`：FreeRTOS V10.3.1、静态分配与互斥锁已启用，内核堆 24 KiB，调用 FromISR 的中断优先级数值必须不小于 5。
- `Inc/stm32f4xx_hal_conf.h`：UART 回调注册已启用，CAN 回调注册未启用。
- `UserCode/Common/vofa.cpp`：当前直接占用 USART6 的 RX/Error HAL 回调和 DMA 收发，必须迁移到统一 UART 实例，避免覆盖回调或并发争用。
- 本地达妙三份手册共同包含 MIT、位置速度、速度、力位混控；JH11 另有周期同步模式，本期不默认扩张到这些模式。旧达妙驱动 `P_MIN/MAX=±π`、`T_MIN/MAX=±16384` 不可直接迁移，量程必须匹配设备配置。
- 本地 Emm V5 手册物理页 35、37–38：默认固定 `0x6B` 校验；速度/位置指令均有 4 字节应答，绝对位置仍有方向字段。旧 ZDT 解析器漏掉命令应答，负位置固定方向亦需修复。
- `Src/usart.c`/`Src/dma.c`/`Src/stm32f4xx_it.c`：USART1/6 有 RX+TX DMA，USART3 仅 RX DMA。当前只有 USART6 开启 UART 全局 IRQ，USART1 的 DMA 发送完成及 IDLE 接收需补全其 IRQ；USART3 RX DMA 优先级 4 不可调用 FreeRTOS FromISR。
- 默认保留 USART6 给 VOFA，USART1 作为 ZDT 可用接入示例，USART3 保留原遥控配置；不自动创建/使能电机，也不把所有 UART 都创建任务。端口检查 RX/TX DMA 链接完整才启动完整双向模式；USART3 不因框架迁移被擅自改波特率或添加冲突 DMA 映射。
- `Src/tim.c`：TIM1 已设 168 分频、周期 20000 tick、四路 PWM，可作为参考工程 50 Hz 舵机的候选资源；初始化核对实际时钟，不改变 LED/Buzzer 共用定时器周期。
- `../2026gcxl/UserCode/BSP/bsp_servo.h/.c`：确认通用 PWM 位置舵机、0–180°/500–2500 µs 参考配置、正反向映射、直接脉宽设置及通道注入。参考实现 `InitEx()` 自动启动、启动后写 CCR，新实现改为显式 Enable 且先写 CCR。
- `../2026gcxl/UserCode/BSP/bsp_tim.h/.c`：APB1/APB2 时钟分组、APB 分频不为 1 时计时器时钟翻倍与 tick/µs 换算。参考使用 `htim->Init.Prescaler`，移植时优先读取运行期 PSC，避免配置缓存与硬件寄存器不一致。
- `tests/host/CMakeLists.txt`：现有算法测试按 `pid.cpp` 搜集，但当前控制器在 `cotr.cpp`；实施先建立基线，修复漏编译后再以既有数值对拍保护命名迁移。
- YOUMI 淘宝页面此前无法读取；参考工程头文件标明其默认标定是通用示例而非 YOUMI 型号规格。按用户最新指定采用该设计与可覆盖的参考预设；示例不自动输出，实际运动前确认所购为定位版本并校准安全端点。

## Steps

- [x] 建立 Debug/Release 与现有主机测试基线，保护用户改动，加入 Google 格式配置并修复测试源列表。
- [x] 整理小型控制能力接口与单位约定，分模块统一手写 C++ 命名及所有调用点，保持既有算法数值行为。
- [x] 实现 UART DMA、FreeRTOS 缓冲和注册回调，补全 IRQ/配置，迁移 VOFA，验证并发、边界和恢复路径。
- [x] 重构必要的 CAN/DJI 边界与命名，移入达妙、ZDT 和确认后的舵机；完善协议测试与安全初始化示例。
- [x] 完成全量主机测试、LSP 检查及 Debug/Release 编译，核对资源增量，整理接线配置和未实测项目。

## Verification

- 协议测试：使用手册独立样例校验 DM 打包/解包、ID/长度/模式与配置量程；ZDT 正负速度/绝对与相对位置、应答/查询、不同 UART 同地址、拆包粘包、噪声重同步与校验；舵机角度到脉宽、反向映射、范围限制、非有限输入、非法通道、不适配周期与使能前不输出。
- 舵机参考预设在 1 MHz 计数、20 ms 周期下，0/π÷2/π rad 对应 CCR 500/1500/2500；覆盖 APB1/APB2 不同分频与自定义标定，不把这些参考值当作厂家实测值。
- UART HAL/RTOS 桩测试：RX DMA 回卷与重复 IDLE/HT/TC 事件、缓冲溢出与解析器重置、多发送者完整消息、DMA Busy/错误/超时、迟到完成事件和回调上下文。桩测试不替代真实 FreeRTOS 调度与硬件验证。
- CAN/DJI 测试：STD/EXT 和远程帧过滤、非法 DLC、满邮箱 Busy；分组同总线/同 TX ID/重复槽验证，不依赖槽 0 存在；目标值生效且每组一次打包。
- 先以只读 LSP 诊断作为静态检查，再运行 `cmake -S tests/host -B tests/host/build`、对应构建与 `ctest --test-dir tests/host/build --output-on-failure`；已有算法对拍与新增驱动测试全部通过。
- STM32 `cmake --preset Debug`/`Release` 与相应 build preset，检查警告、Flash/RAM、每总线任务栈、24 KiB 内核堆余量和 DMA 地址。
- 硬件验收：VOFA 输出不回归，串口环回与多任务压力，ZDT 查询/速度/正负位置，达妙使能/模式/反馈，示波器确认舵机脉宽。没有实物不声称这些已通过；上电默认不使能电机。

## 已确定的边界

- 舵机设计来源已由用户指定为 `2026gcxl`；使用可覆盖的参考预设，不等待淘宝规格即可实现，真实硬件参数与安全端点留到上板验收。
- UART、CAN 必要重构与全工程自有 C++ 风格统一已授权；不改第三方/CubeMX 外部 API，不引入无关框架或控制算法重写。
- 达妙本期默认 MIT 扭矩与完整 MIT 命令、共同控制/反馈协议；原生位置速度和速度模式保留明确协议 API，不将模式不匹配的对象伪装为支持所有能力，不扩张到 JH11 专属同步模式。
- 不自动创建新电机并运动；串口与定时器映射由调用方显式指定。硬件尚未验证的参数、可靠性与资源指标不得写成已实测结果。
