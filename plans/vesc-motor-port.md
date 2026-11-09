# VESC 驱动移植

## Context

把 QX_CK 的 VESC 驱动（`Project/Hardware_Drivers/Motor_VESC.c/.h`）移植进当前工程。QX_CK 是 H7+FDCAN，本工程是 F407+bxCAN，且有既有的 `CanNode` 与能力接口体系，因此按现有架构重写，保持实现尽量简单。

需求已定：**四种控制模式全做 + 解析 STATUS_1 + 预置 M3508/M2006 + 允许最小扩展 CanNode**。

## VESC 协议（已核实，来源 vedderb/bldc `comm_can.md` 与 `comm/comm_can.c`）

扩展帧 29 位：`ext_id = (command << 8) | controller_id`

| 命令 | ID | 载荷 | 缩放 |
|---|---|---|---|
| SET_DUTY | 0 | int32 | duty × 100000 |
| SET_CURRENT | 1 | int32 | A × 1000 |
| SET_CURRENT_BRAKE | 2 | int32 | A × 1000 |
| SET_RPM | 3 | int32 | ERPM（机械RPM × 极对数） |
| SET_POS | 4 | int32 | 转子角度(度) × 1000000 |

反馈 `CAN_PACKET_STATUS`(9)：`ext_id = 0x900 | controller_id`

| 偏移 | 类型 | 含义 |
|---|---|---|
| 0..3 | int32 | ERPM |
| 4..5 | int16 | 电流，**÷10 得 A** |
| 6..7 | int16 | duty，÷1000 |

全部多字节字段为大端（QX_CK 的 `buffer_append_int32` 同样是高字节在前）。

## 与 QX_CK 的差异（移植时修正）

1. **FDCAN → bxCAN**：改用 `HAL_CAN_AddTxMessage` + `CAN_ID_EXT`。
2. **QX_CK 反馈电流漏除 10**：它把 `Data[4..5]` 直接当扭矩存；官方是 `int16/10.0`，本次按官方实现。
3. **QX_CK 丢弃 duty**：`Data[6..7]` 未解析，本次补上。

## CanNode 最小扩展（两处）

1. `hal_can.hpp`：新增 `canCfg::kExtIdMaskFull = 0x1FFFFFFF`。现有 `kIdMaskNone = 0x7FF` 只能匹配标准帧，而 VESC 反馈 ID 是 `0x900|id`。
2. `hal_can.hpp/.cpp`：新增受保护方法 `SendExtendedWithId(uint32_t ext_id, const uint8_t* data, uint16_t len)`。现有 `SendExtended` 用的是构造期固定的 `id_`，而 VESC 每条命令的高字节不同。

`FilterInit` 已是 mask=0 全放行，接收分发已按 `hdr.IDE == CAN_ID_EXT` 取 `ExtId`（`hal_can.cpp:92`），硬件层无需改动。

## 设计（保持简单）

```cpp
struct VescSpec {
    uint8_t pole_pairs;            // 极对数
    float   gear_ratio;            // 转子→输出轴
    float   kt_output_nm_per_a;    // 输出轴 N·m/A（手册转矩常数）
    float   current_max_a;         // 电流限幅
    uint32_t erpm_max;             // 电转速限幅
};

class VescMotor final : public motor::ITorqueControl,
                        public motor::IVelocityControl,
                        public motor::IPositionControl,
                        public CanNode {
    VescMotor(CAN_HandleTypeDef* hcan, uint8_t controller_id, const VescSpec& spec);

    // 能力接口
    SetTargetTorqueNm / SendTorque        // 扭矩 → 电流(÷kt) → SET_CURRENT
    SetTargetVelocityDegPerSec / SendVelocity  // 度/秒 → ERPM → SET_RPM
    SetTargetPositionDeg / SendPosition   // 输出轴度 → 转子度 → SET_POS

    // 专有命令（不属于能力接口）
    Status SetDuty(float duty);           // SET_DUTY
    Status SetBrakeCurrent(float amp);    // SET_CURRENT_BRAKE

    // STATUS_1 反馈
    float VelocityDegPerSec() const;      // ERPM ÷极对数 ÷gear ×6
    float CurrentA() const;
    float Duty() const;
};
```

反馈只做 STATUS_1 的三个量，**不提供位置反馈**（协议本身没有），因此 `IPositionControl` 只下发目标，`SendPosition` 返回的是"已接受/已发送"。

构造时把 `feedback_id` 设为 `0x900 | controller_id`、掩码用 `kExtIdMaskFull`，直接复用 `CanNode` 的中断分发。

## 预设（数值取自本地 RM 手册）

| 预设 | 极对数 | 减速比 | 转矩常数（输出轴） | 依据 |
|---|---|---|---|---|
| `kVescM3508` | 7 | 3591/187 | 0.3 N·m/A | M3508 手册 |
| `kVescM2006` | 7 | 36 | 0.18 N·m/A | M2006 手册 |

电流限幅按 C620/C610 量程给初值（M3508 20 A、M2006 10 A），ERPM 上限按额定转速折算，注释标注需按实机校准。

## Files to modify

- `UserCode/HAL/hal_can.hpp/.cpp`：新增扩展帧全掩码常量与 `SendExtendedWithId`。
- `UserCode/DRV/motor/drv_vesc_motor.hpp/.cpp`（新增）：`VescSpec`、两个预设、`VescMotor`。
- `UserCode/APP/motor_demo.hpp`：补 VESC 用法示例（不自动使能）。
- 不改 DJI / 达妙 / ZDT / 舵机驱动。

## Reuse

- `CanNode` 注册、中断分发、`OnFrame` 回调（`hal_can.hpp/.cpp`）。
- `canCfg::kExtIdMask = 0x1FFFFFFF`（`hal_can.hpp:23`）。
- 能力接口与度制约定（`motor_base.hpp`）。
- 大端打包：`utils::PutBeI16`（`utils.hpp`）与新增的 int32 大端写法。

## Steps

- [x] `hal_can`：加 `kExtIdMaskFull` 与 `SendExtendedWithId`。
- [x] 新建 `VescSpec` 与 `kVescM3508` / `kVescM2006` 预设。
- [x] 实现四种命令打包（duty / current / brake / rpm / pos）。
- [x] 实现 STATUS_1 解包（ERPM、电流÷10、duty÷1000）。
- [x] 实现三个能力接口 + duty/brake 专有方法。
- [x] `motor_demo` 补示例。
- [x] 主机测试：帧字节与缩放、ERPM↔度/秒、电流↔扭矩。

## Verification

- `cmake --build --preset Debug` / `Release`：0 错误 0 警告。
- `ctest --test-dir tests/host/build`：7/7 通过。
- 新增主机测试：
  - `SET_CURRENT(1.0 A)` → `ext_id=(1<<8)|id`，payload `00 00 03 E8`
  - `SET_DUTY(0.5)` → payload `00 00 C3 50`（50000）
  - `SET_POS(90°转子)` → payload `05 5D 4A 80`（90×1e6）
  - STATUS_1 解包：ERPM=1000、电流原始 123（→12.3 A）、duty 原始 500（→0.5）
- `clangd --check` 无代码诊断。
- 上板（未做不得声称通过）：与真实 VESC 联调四种命令与 STATUS 反馈。

## 已确定

1. 四种控制模式全做（duty / current / brake / rpm / pos）。
2. 反馈只解析 STATUS_1。
3. 预设 M3508、M2006（手册值）。
4. 允许对 `CanNode` 做最小扩展（掩码常量 + 显式 ID 发送）。
