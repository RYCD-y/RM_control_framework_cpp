# 把 control_framework_v2 的算法模块引入 Common 并 C++ 化

> 已确认：**方案 B（完全类化）** ／ kalman 保留 `malloc`（2a）／ **OLS + DWT 一起移植** ／ **host 对拍验证**。

## Context

`three_friction` 目前只有 CAN/电机驱动，缺控制算法层。把 `control_framework_v2` 里成熟的
PID/前馈/LDOB/TD + 卡尔曼 + 四元数 EKF + CRC 搬进 `three_friction/UserCode/Common/`，
C 风格的 `struct + 自由函数` 改写为 **class**（构造配置、方法成员化、`#define` → `constexpr`）。

### 源文件与依赖（已 grep 确认）

| 源文件 | 行数 | 依赖 |
|---|---|---|
| `controller.h/.c` | 275 / 661 | `OLS_*`（utils.c）、`DWT_GetDeltaT`（hal_dwt.c）、`math.h`；`arm_math.h` 只 include、**0 处调用**（可去掉） |
| `utils.h/.c`（只需 OLS 部分） | ~150 | `user_malloc`、`memset` |
| `hal_dwt.h/.c` | 44 / 122 | `main.h`、`DWT->CYCCNT` |
| `kalman_filter.h/.c` | 121 / 480 | CMSIS `arm_matrix_instance_f32` + `arm_mat_*`、`user_malloc` ×27 |
| `QuaternionEKF.h/.c` | 75 / 492 | `kalman_filter`、`atan2f/asinf` |
| `crc8_crc16.h/.c` | 118 / 282 | 无 |

`three_friction` 现有可复用：`UserCode/Common/utils.hpp`（`namespace utils` 的 `float_to_signed/be_i16`）、
CMSIS-DSP 已链接、FreeRTOS `pvPortMalloc`、`HAL_GetTick()`（1 ms，**不够做 PID dt，故移植 DWT**）。

## Approach

### 类划分（一个模块一个类/命名空间）

```
UserCode/Common/
├── dwt.hpp / dwt.cpp                     class Dwt            （全静态，硬件计时封装）
├── ols.hpp / ols.cpp                     class OrdinaryLeastSquares
├── crc.hpp / crc.cpp                     namespace crc        （无状态，inline 表）
├── pid.hpp / pid.cpp                     class Pid / FuzzyRule / Feedforward / Ldob / TrackingDifferentiator
├── kalman_filter.hpp / kalman_filter.cpp class KalmanFilter
└── quaternion_ekf.hpp / quaternion_ekf.cpp class QuaternionEkf
```

命名约定：新代码用 PascalCase 类名 + 成员函数小驼峰（与现有 `djiMotor` 风格不同，但这是算法层，
和驱动层分开；若要统一可后续再改）。

### 封装原则（参数尽量隐藏）

1. **所有状态字段放 `private`**，只在构造函数/`init()` 里由配置参数写入；
2. 对外只暴露三类接口：**配置**（init / setter）、**计算**（calculate / update）、**只读查询**（getter）；
3. 需要在线调整的运行参数（PID 的 kp/ki/kd、限幅）单独给 `setXxx()`，不做公开字段；
4. 需要外部注入的矩阵/表（KF 的 F/H/Q/R、Fuzzy 规则表）用**语义化 setter**，不暴露裸指针成员；
5. 算法内部中间量一律私有，getter 只给"外部真正会读"的那几个（输出、积分项、状态向量）；
6. 对拍测试同样只走 public 接口，不靠 `friend` 或公开字段——这也是对封装是否够用的检验。

### 关键接口（草案）

```cpp
// ── dwt.hpp ──────────────────────────────────────────────
class Dwt
{
public:
        static void  init(uint32_t cpu_freq_mhz);
        static float deltaT(uint32_t& cnt_last);     // 替代 DWT_GetDeltaT(uint32_t*)
        static float timelineS();
        static float timelineMs();
        static uint64_t timelineUs();
        static void  delay(float seconds);
        static void  updateSysTime();
};
```

```cpp
// ── ols.hpp ──────────────────────────────────────────────
class OrdinaryLeastSquares
{
public:
        void  init(uint16_t order);                  // 内部 pvPortMalloc（固件）/ malloc（host）
        void  update(float deltax, float y);
        float derivative(float deltax, float y);     // 更新并返回导数
        float smooth(float deltax, float y);         // 更新并返回平滑值
        float slope() const { return k_; }           // 只读：当前拟合斜率
        float intercept() const { return b_; }
        float stdDev() const { return std_dev_; }
        uint16_t order() const { return order_; }
        uint32_t count() const { return count_; }

private:
        uint16_t order_ = 0;
        uint32_t count_ = 0;
        float   *x_ = nullptr;      // init 时 pvPortMalloc
        float   *y_ = nullptr;
        float    k_ = 0.0f, b_ = 0.0f, std_dev_ = 0.0f;
        float    t_[4] = {};
};
```

```cpp
// ── pid.hpp ──────────────────────────────────────────────
class Pid
{
public:
        using Hook = void (*)(Pid &);                // 替代 void (*)(struct pid_t*)，零开销

        enum Improve : uint8_t
        {
                None = 0x00, IntegralLimit = 0x01, DerivativeOnMeasurement = 0x02,
                TrapezoidIntegral = 0x04, ProportionalOnMeasurement = 0x08,
                OutputFilter = 0x10, ChangingIntegrationRate = 0x20,
                DerivativeFilter = 0x40, ErrorHandle = 0x80,
        };

        // 配置（一次性）
        void init(float max_out, float integral_limit, float deadband,
                  float kp, float ki, float kd, float a, float b,
                  float output_lpf_rc, float derivative_lpf_rc,
                  uint16_t ols_order, uint8_t improve);

        // 运行期可调
        void setGains(float kp, float ki, float kd);
        void setLimits(float max_out, float integral_limit, float deadband);
        void setHooks(Hook func1, Hook func2);

        // 计算
        float calculate(float measure, float ref);
        void  reset();

        // 只读查询（监控/对拍用）
        float output() const { return output_; }
        float pOut() const { return p_out_; }
        float iOut() const { return i_out_; }
        float dOut() const { return d_out_; }
        float iTerm() const { return i_term_; }
        float err() const { return err_; }
        float dt() const { return dt_; }
        bool  blocked() const { return error_type_ != 0; }

private:
        float ref_ = 0, kp_ = 0, ki_ = 0, kd_ = 0;
        float measure_ = 0, last_measure_ = 0, err_ = 0, last_err_ = 0, last_i_term_ = 0;
        float p_out_ = 0, i_out_ = 0, d_out_ = 0, i_term_ = 0;
        float output_ = 0, last_output_ = 0, last_d_out_ = 0;
        float max_out_ = 0, integral_limit_ = 0, deadband_ = 0, control_period_ = 0;
        float coef_a_ = 0, coef_b_ = 0, output_lpf_rc_ = 0, derivative_lpf_rc_ = 0;
        uint32_t dwt_cnt_ = 0;
        float    dt_ = 0;
        uint8_t  improve_ = 0;
        uint8_t  error_type_ = 0;
        uint64_t error_count_ = 0;
        FuzzyRule fuzzy_;                  // 值成员（v2 是指针，这里收进内部）
        OrdinaryLeastSquares ols_;
        Hook func1_ = nullptr, func2_ = nullptr;
};
```

`FuzzyRule` / `Feedforward` / `Ldob` / `TrackingDifferentiator` 同样处理：字段全 private，
`init()` 配置、`calculate()` 计算、少量 getter。四个类都放 `pid.hpp`（v2 里它们同属 controller.h，互相无依赖）。

```cpp
// ── kalman_filter.hpp ────────────────────────────────────
class KalmanFilter
{
public:
        void init(uint8_t xhat_size, uint8_t u_size, uint8_t z_size);   // 内部 kfAlloc()

        // 语义化注入（v2 是直接写 kf.F_data[...]，这里收进 setter）
        void setStateTransition(const float *f);          // xhatSize²
        void setMeasurementMatrix(const float *h);        // zSize×xhatSize
        void setMeasurementMap(const uint8_t *map);       // zSize
        void setMeasurementDegree(const float *deg);      // zSize
        void setProcessNoise(const float *q);             // xhatSize²
        void setMeasurementNoiseDiag(const float *r_diag); // zSize
        void setStateMinVariance(const float *v);         // xhatSize
        void setInitialState(const float *x0);            // xhatSize
        void setInitialCovariance(const float *p0);       // xhatSize²
        void setControlMatrix(const float *b);            // xhatSize×uSize（uSize!=0 时）
        void setMeasurementValid(uint8_t idx, bool valid);

        // 计算
        void  measure();
        void  xhatMinusUpdate();
        void  pMinusUpdate();
        void  setK();
        void  xhatUpdate();
        void  pUpdate();
        const float *update();                   // 返回 xhat 数据（替代 float* 裸指针）

        // 只读查询
        const float *state() const;              // xhat
        const float *covariance() const;         // P
        float filteredValue(uint8_t i) const;

private:
        // 矩阵实例与缓冲区全部私有（kfAlloc 分配）
        arm_matrix_instance_f32 xhat, xhatminus, u, z, P, Pminus, F, FT, B, H, HT, Q, R, K, S;
        float *xhat_data, *xhatminus_data, *u_data, *z_data, *P_data, *Pminus_data;
        float *F_data, *FT_data, *B_data, *H_data, *HT_data, *Q_data, *R_data, *K_data, *S_data;
        uint8_t *MeasurementMap, *temp;
        uint8_t xhat_size_ = 0, u_size_ = 0, z_size_ = 0;
        uint8_t SkipEq1_ = 0, SkipEq2_ = 0, SkipEq3_ = 0, SkipEq4_ = 0, SkipEq5_ = 0;
        friend class QuaternionEkf;              // EKF 需要直接改 F/H/Q/R 的内部值
};
```

`kfAlloc()` 的落点（保持 malloc 语义）：

```cpp
// kalman_filter.hpp
#if defined(USE_FREERTOS_HEAP)
#include "FreeRTOS.h"
inline void *kfAlloc(size_t n) { return pvPortMalloc(n); }
#else
inline void *kfAlloc(size_t n) { return malloc(n); }     // host 对拍
#endif
```

```cpp
// ── quaternion_ekf.hpp ───────────────────────────────────
class QuaternionEkf
{
public:
        void init(float q1, float q2, float r, float lambda, float lpf);
        void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

        // 只读查询（v2 用全局 QEKF_INS 读这些值）
        const float *quaternion() const { return q_; }
        const float *gyroBias() const { return gyro_bias_; }
        float roll() const { return roll_; }
        float pitch() const { return pitch_; }
        float yaw() const { return yaw_; }
        float yawTotalAngle() const { return yaw_total_angle_; }
        bool  converged() const { return converge_flag_ != 0; }

private:
        KalmanFilter kf_;
        float q_[4] = {}, gyro_bias_[3] = {};
        float gyro_[3] = {}, accel_[3] = {};
        float roll_ = 0, pitch_ = 0, yaw_ = 0, yaw_total_angle_ = 0;
        float q1_, q2_, r_, lambda_, dt_;
        uint8_t converge_flag_ = 0, stable_flag_ = 0, initialized_ = 0;
        ... (chi-square / LPF 系数等内部量)
};
```

不设 `extern` 全局实例（v2 的 `QEKF_INS`），由使用方定义 `static QuaternionEkf g_ekf;`——更 C++，
对拍时两边各定义一个。

### 从 utils 只引入真正需要的三个东西

`grep` 统计 controller.c 对 utils 的引用，只有 **`sign`(10 次)、`float_limit`(2 次)、OLS 系列**；
`kalman_filter.c` / `QuaternionEKF.c` **完全不用 utils**。所以：

| utils 里的东西 | 处理方式 |
|---|---|
| `OrdinaryLeastSquares_t` + `OLS_*`（5 个函数，~150 行） | **引入**，改成 `class OrdinaryLeastSquares`（`ols.hpp/.cpp`） |
| `sign()` | **引入**，加进现有 `namespace utils`（`utils.hpp`）：`inline float sign(float v) { return v >= 0.0f ? 1.0f : -1.0f; }` |
| `float_limit()` / `abs_limit()` | **不引入**，实现与 `std::clamp` 完全等价（已逐行核对），直接用 `std::clamp` |
| `Sqrt / ramp_* / theta_format / float_deadband / loop_float_limit / float_to_int16 / VAL_MAX / VAL_MIN / rad_format / OLS 之外的其余函数` | **不引入**（本批模块一处都没用到） |

`float_deadband()` 若要保留（PID 的 DeadBand 字段用的是内联写法，不需要它），也不引入。

## 对拍验证（host）

新建 `tests/host/`（**不参与固件构建**：CMake 的 GLOB 只扫 `UserCode/**`）。

```
tests/host/
├── CMakeLists.txt          # g++ 构建，include 两边源码
├── stubs/
│   ├── arm_math.h          # arm_matrix_instance_f32 + arm_mat_* 声明
│   ├── arm_math_stub.cpp   # 与 CMSIS 同累加顺序的浮点实现
│   ├── FreeRTOS.h          # pvPortMalloc → malloc
│   ├── hal_dwt_host.h/.cpp # 可控时间步进的 DWT 替身（两边共用）
│   └── utils_host.h/.cpp   # v2 的 utils（OLS 之外的部分）host 版
├── ref/                    # 直接编译 v2 的 .c（引用绝对路径，不复制）
└── test_{crc,ols,dwt,pid,kf,ekf}.cpp
```

对拍方式：同一输入序列喂给 `ref`（v2 的 `.c`）和 `new`（`.cpp` 类），逐项比较。

**比较只用 public 接口**（这正是封装的验收）：`Crc8()/Crc16()` 返回值、`Ols::derivative()/smooth()/slope()`、
`Pid::output()/iTerm()/dOut()`、`KalmanFilter::state()/covariance()`、`QuaternionEkf::quaternion()/roll()/pitch()/yaw()`。
v2 侧直接读结构体字段，C++ 侧读 getter，两边值一一对应。

| 模块 | 用例 | 容差 |
|---|---|---|
| CRC | 100 组随机长度/内容 | 完全相等 |
| OLS | 500 点含噪序列，比较 `derivative/smooth/k/b` | bit-exact |
| DWT | 固定 CYCCNT 步进 | bit-exact |
| Pid | 阶跃 + 正弦；分别打开 8 个 Improve 位；含 Fuzzy | bit-exact |
| Feedforward / Ldob / TD | 斜坡 + 阶跃序列 | bit-exact |
| KalmanFilter | 固定 F/H/Q/R，200 步 `update()`，比较 `xhat` 与 `P` | 1e-5 相对误差（host stub 与 CMSIS 累加顺序可能不同） |
| QuaternionEkf | 1000 步恒定陀螺 + 静态加速度，比较 `q/GyroBias/Roll/Pitch/Yaw` | 1e-5 |

> 板端最终都链接同一个 `libarm_cortexM4lf_math.a`，所以 C 版与 C++ 版在目标机上应当 bit-exact；
> host 的 1e-5 只是因为 stub 的浮点累加顺序。

## Steps

- [x] 1. 建 `tests/host/`：CMakeLists + `arm_math/FreeRTOS/hal_dwt/utils` 四个 stub，先把 **v2 的 .c 编过并跑通**（作为基准，stub 的 utils 只需 `sign/OLS_*`）
- [x] 2. `dwt.hpp/.cpp`（class Dwt）→ `test_dwt.cpp` 对拍
- [x] 3. `ols.hpp/.cpp`（class OrdinaryLeastSquares，保留 pvPortMalloc）→ `test_ols.cpp` 对拍
- [x] 4. `crc.hpp/.cpp`（namespace crc，`inline constexpr` 表）→ `test_crc.cpp` 对拍
- [x] 5. `pid.hpp/.cpp`（Pid / FuzzyRule / Feedforward / Ldob / TrackingDifferentiator）→ `test_pid.cpp` 对拍（8 个 Improve 位逐个）
- [x] 6. `kalman_filter.hpp/.cpp`（class KalmanFilter，kfAlloc 保留 malloc）→ `test_kf.cpp` 对拍
- [x] 7. `quaternion_ekf.hpp/.cpp`（class QuaternionEkf，含 KalmanFilter 成员）→ `test_ekf.cpp` 对拍
- [x] 8. `utils.hpp` 只追加一个 `sign()`（不引入 utils.c 的其余函数）
- [x] 9. 固件集成：`ninja -C build/Debug` / `Release` 0 warning，记录 Flash/RAM 增量

## Files to add

```
three_friction/UserCode/Common/{dwt,ols,crc,pid,kalman_filter,quaternion_ekf}.{hpp,cpp}
three_friction/tests/host/{CMakeLists.txt,stubs/*,test_*.cpp}
```

`three_friction/UserCode/Common/utils.hpp` 追加 `sign()` / `deadband()` 两个 inline 函数。

## Reuse

| 复用 | 位置 |
|---|---|
| `utils::float_to_signed / int_to_float / be_i16 / put_be_i16` | `three_friction/UserCode/Common/utils.hpp` |
| `std::clamp` 替代 `float_limit/abs_limit`（已逐行核对等价） | `<algorithm>`（C++17，已用于 `djiMotor::torqueToRaw`） |
| `sign()`（需要新增到 `namespace utils`） | `three_friction/UserCode/Common/utils.hpp` |
| CMSIS-DSP `arm_mat_*` | 已链接的 `libarm_cortexM4lf_math.a` |
| `pvPortMalloc` | FreeRTOS `heap_4.c` |
| 原算法实现（逐行对照） | `control_framework_v2/UserCode/Common/{controller,utils,kalman_filter,QuaternionEKF}.c`、`UserCode/HAL/hal_dwt.c` |

**不引入**（核对过本批模块一处都没用）：`Sqrt / ramp_init / ramp_calc / theta_format / float_deadband /
loop_float_limit / float_rounding / float_to_int16 / int16_to_float / int16_limit / VAL_MAX / VAL_MIN / rad_format`。

## Verification

1. `cd tests/host && cmake -B build && cmake --build build && ctest` → 全部对拍用例通过
2. `ninja -C build/Debug` / `ninja -C build/Release` → 0 error / 0 warning
3. `arm-none-eabi-size` 对比引入前后：预期 Flash +6~10 KB（arm_math 已链接），RAM 增量 = EKF 的 malloc 占用（约 1 KB）
4. 抽查一个真实场景：用 `djiMotor::getSpeed()` 作 PID 反馈、`setTorque()` 作输出，串起来跑一圈看是否收敛（板端，可选）
