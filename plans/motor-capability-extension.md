# motor_base 能力扩展：MIT 与使能控制

## Context

现状 `motor_base.hpp` 只有三个最小接口（扭矩/速度/位置），而"MIT 控制"与"使能/失能"只存在于具体驱动（`DmMotor`、`ZdtMotor`、`PwmServo`）。上层拿到 `IPositionControl*` 就无法调用这些能力。

本次目标：把 **MIT 控制**与**使能/失能**提升为能力接口，让上层按能力调用。

**本次不动"带全参数的位置/速度控制"（原 Q3）**：达妙的 `SendPositionVelocity()` / `SendVelocity()` 保持为 `DmMotor` 的具体方法，不进入接口。

### 现有能力矩阵（已核对）

| 能力 | DJI | 达妙 | ZDT | 舵机 |
|---|---|---|---|---|
| MIT | ❌ | ✅ `SendMit` | ❌ | ❌ |
| 使能/失能 | ❌ **无软件命令** | ✅ | ✅（带 clear 参数） | ✅ |
| 全参数位置/速度 | ❌ | ✅（本次不入接口） | ⚠️ 用默认值 | ❌ |

关键约束：
- MIT 只有达妙支持；`DmMitCommand` 已定义于 `drv_dm_motor.hpp:22`。
- C620/GM6020 **没有软件使能命令**，故 DJI 不实现使能接口（Q2 结论）。
- 当前 `Enable()` 返回类型三个驱动不一致：`HAL_StatusTypeDef`（达妙、舵机）与 `motor::Status`（ZDT）。
  → **全部映射为 `motor::Status`**。
- 单位：位置/速度/kp/kd 全部为**度制**，接口中不得出现弧度。

### 返回类型映射（`HAL_StatusTypeDef` → `motor::Status`）

| HAL 结果 | `motor::Status` | 典型场景 |
|---|---|---|
| `HAL_OK` | `kOk` | 命令已发出 |
| `HAL_BUSY` | `kBusy` | 外设正忙 / 队列满 |
| `HAL_TIMEOUT` | `kIoError` | 发送/等待超时 |
| `HAL_ERROR` | `kIoError` | 外设拒绝或总线错误 |
| `nullptr` / 未 `Init()` | `kNotReady` | 未初始化 |
| 参数非法 | `kInvalidArgument` | 有限性/范围校验失败 |
| 硬件无此能力 | `kUnsupported` | 保留给后续驱动 |

现有 6 个枚举值（`kOk`/`kBusy`/`kUnsupported`/`kInvalidArgument`/`kNotReady`/`kIoError`）已能覆盖上述情形，**无需新增枚举值**；若实现中发现某个错误无法归类，再单独追加（不改动已有值）。

## Approach

延续"能力接口彼此独立"原则，新增**两个**小接口，驱动按实际能力选择继承：

```cpp
namespace motor {

// MIT 五参数命令（位置/速度/kp/kd/前馈力矩），全部度制
struct MitCommand {
    float position_deg = 0.0f;
    float velocity_deg_per_sec = 0.0f;
    float kp = 0.0f;                      // N·m/deg
    float kd = 0.0f;                      // N·m·s/deg
    float feedforward_torque_nm = 0.0f;
};

class IEnableControl {
public:
    virtual ~IEnableControl() = default;
    virtual Status Enable() = 0;
    virtual Status Disable() = 0;
};

class IMitControl {
public:
    virtual ~IMitControl() = default;
    virtual Status SendMit(const MitCommand& command) = 0;
};

}  // namespace motor
```

`Enable/Disable` 统一返回 `motor::Status`。是否已使能的查询不放进接口（避免强制每个驱动维护状态），继续由各具体类的 `IsEnabled()` 提供。

### kp/kd 的度制换算

达妙协议内部按弧度定义刚度/阻尼（`kp_max=500` 对应 N·m/rad）。接口层改用度制后，换算关系为：

- 用力矩等式对齐：`kp_rad × Δpos_rad = kp_deg × Δpos_deg`，且 `Δpos_rad = Δpos_deg × π/180`
- 因此 `kp_rad = kp_deg × 180/π`，`kd_rad = kd_deg × 180/π`

`DmMotor::SendMit` 内部先做该换算，再按协议 `kp_max`/`kd_max` 钳位与 12 位映射。
复用现有 `kDegToRad`（`drv_dm_motor.cpp:12`），换算写成 `value / kDegToRad`。
度制下等效上限约 `kp≈8.727`、`kd≈0.0873`，`DmMotorSpec` 的 `kp_max/kd_max` 保持协议原值（500/5）不变。

DJI **不实现** `IEnableControl`，也不需要返回 `kUnsupported`——编译期就无法把它当成可使能设备使用，比运行时才发现更早暴露错误。

## Files to modify

- `UserCode/DRV/motor/motor_base.hpp`：新增 `MitCommand`、`IEnableControl`、`IMitControl`。
- `UserCode/DRV/motor/drv_dm_motor.hpp/.cpp`：继承两个新接口；`SendMit` 改用 `motor::MitCommand`；`Enable/Disable` 返回 `motor::Status`；保留 `using DmMitCommand = motor::MitCommand;` 兼容旧名。
- `UserCode/DRV/motor/drv_zdt_motor.hpp/.cpp`：实现 `IEnableControl`，`Enable()`/`Disable()` 转调现有 `Enable(bool, bool)`。
- `UserCode/DRV/motor/drv_servo.hpp/.cpp`：实现 `IEnableControl`，`Enable/Disable` 返回 `motor::Status`。
- `UserCode/DRV/motor/drv_dji_motor.hpp/.cpp`：不改。
- `UserCode/APP/motor_demo.hpp/.cpp`：加一段"检查能力再调用"的示例。

## Reuse

- `DmMotor::SendMit`（`drv_dm_motor.cpp`）的打包逻辑完全复用，只替换参数类型，并在入口增加 kp/kd 的度→协议制换算。
- `DmMotor::Enable/Disable`、`ZdtMotor::Enable(bool,bool)`、`PwmServo::Enable/Disable` 已有实现，仅做返回类型与签名适配。
- `PwmServo::IsEnabled()`（`drv_servo.hpp:46`）保留。
- `kDegToRad`（`drv_dm_motor.cpp:12`）继续在 MIT 打包内部使用：位置/速度用 `× kDegToRad`，kp/kd 用 `/ kDegToRad`。

## Steps

- [x] `motor_base.hpp`：新增 `MitCommand`（全度制）、`IEnableControl`、`IMitControl`。
- [x] `DmMotor`：继承两接口，`SendMit(motor::MitCommand)` 并加入 kp/kd 度→协议制换算，`Enable/Disable` 返回 `motor::Status`，保留旧类型别名。
- [x] `ZdtMotor`：继承 `IEnableControl`，`Enable()`/`Disable()` 转调 `Enable(bool, bool)`，HAL 结果映射为 `motor::Status`。
- [x] `PwmServo`：继承 `IEnableControl`，`Enable/Disable` 返回类型统一为 `motor::Status`。
- [x] `motor_demo`：新增按能力分发的示例（`IEnableControl` / `IMitControl`）。
- [x] 全工程检查 `Enable/Disable` 调用点并同步。

## Verification

- `cmake --build --preset Debug` / `Release`：0 错误 0 警告。
- `ctest --test-dir tests/host/build`：7/7 通过。
- `clangd --check` 对 `motor_base.hpp` 与三个驱动无代码诊断。
- 主机侧纯逻辑测试（可直接测）：MIT 度→协议制换算与钳位、位置/速度度→弧度换算。
- 检查 `IMitControl`/`IEnableControl` 的 vtable 已生成、无未定义符号（`arm-none-eabi-nm`）。
- 上板（未做不得声称通过）：达妙 MIT 跟随、ZDT/舵机使能与失能。

## 已确定

1. Q1 → 独立能力接口（不做全能力基类）。
2. Q2 → DJI 不实现 `IEnableControl`。
3. Q3 → 本次不改，`SendPositionVelocity`/`SendVelocity` 保持为达妙具体方法。
4. 反馈① → 所有 `Enable/Disable` 统一返回 `motor::Status`，并附 HAL→Status 映射表；评估现有枚举足够，不新增值。
5. 反馈② → MIT 的 kp/kd 也用度制（N·m/deg、N·m·s/deg），驱动内部换算为协议弧度制。
