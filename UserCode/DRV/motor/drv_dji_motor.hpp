//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_DRV_DJI_MOTOR_HPP
#define THREE_FRICTION_DRV_DJI_MOTOR_HPP

#include <stdint.h>

#include "hal_can.hpp"
#include "motor_base.hpp"
#include "utils.hpp"

// 控制量物理单位：电流(A) 或 电压(V)
enum class DjiCtrlMode : uint8_t
{
        Current,
        Voltage
};

// ─────────────── DJI 电机参数（数值取自 RoboMaster 官方手册）───────────────
//
//   M3508 P19 + C620 : 额定扭矩 3 N·m @ 额定电流 10 A，Torque Constant 0.3 N·m/A（输出轴），
//                      减速比 3591/187 ≈ 19.2032，额定转速 469 rpm（输出轴）
//   M2006 P36 + C610 : 额定扭矩 1 N·m @ 额定电流 3 A（→ 0.33333 N·m/A 输出轴），
//                      减速比 36:1，额定转速 416 rpm（输出轴）
//   GM6020           : 额定扭矩 1.2 N·m @ 额定电流 1.62 A（→ 0.74074 N·m/A），无减速箱
//   C620 控制量程    : -16384~16384 ↔ -20~20 A
//   C610 控制量程    : -10000~10000 ↔ -10~10 A
//   GM6020 电流模式  : -16384~16384 ↔ -3~3 A
//   GM6020 电压模式  : -25000~25000 ↔ -24~24 V
//
// 帧结构：一个控制帧（8 字节）带同组 4 台电机，每台 2 字节大端。
//   C620/C610 : 0x200 → 反馈ID 0x201~0x204 ; 0x1FF → 0x205~0x208 ; 0x1FE → 0x209~0x20B
//   GM6020    : 0x1FF → 0x205~0x208 ; 0x2FF → 0x209~0x20B
//
// 关键设计：kt_rotor 存【转子侧】力矩常数，不含减速箱。
// 输出轴力矩 = kt_rotor * gear_ratio。
// 于是"装减速箱 / 不装减速箱"只是 gear_ratio 一个字段的差别（见 withGearRatio）。
struct djiMotorSpec
{
        const char *name;
        DjiCtrlMode mode;

        float       gear_ratio;     // 减速比，1.0f = 直连（不装减速箱）

        float       ctrl_raw_full;  // 控制量满量程 raw
        float       ctrl_unit_full; // 控制量满量程物理量（A 或 V）

        float       fb_raw_full;    // 反馈电流满量程 raw
        float       fb_unit_full;   // 反馈电流满量程物理量（A）

        float       kt_rotor;       // 转子侧力矩常数 N·m/A（不含减速箱）
        bool        angle_is_absolute;

        // 以下派生量随 gear_ratio 自动变化
        constexpr float ktOutput() const { return kt_rotor * gear_ratio; }
        constexpr float maxTorque() const { return fb_unit_full * ktOutput(); }
        // 反馈转速 LSB(=1 rpm，转子侧) → 输出轴 rpm
        constexpr float speedScale() const { return 1.0f / gear_ratio; }
        // 反馈角度 LSB(=360/8192 度，转子侧) → 输出轴 度
        constexpr float angleScale() const { return 360.0f / 8192.0f / gear_ratio; }
};

// 换减速箱：同一个电机，只改 gear_ratio 和显示名
constexpr djiMotorSpec withGearRatio(djiMotorSpec spec, float ratio, const char *name)
{
        spec.gear_ratio = ratio;
        spec.name       = name;
        return spec;
}

// 在已有减速比上再叠加一级外部减速（同步带/行星轮）：gear_ratio 相乘
constexpr djiMotorSpec withExtraGear(djiMotorSpec spec, float extra_ratio, const char *name)
{
        spec.gear_ratio *= extra_ratio;
        spec.name        = name;
        return spec;
}

// ── 拆掉原厂减速箱、转子直驱（gear_ratio = 1.0）──
// 注意：M3508/M2006 是集成减速电机，减速箱不可"选装"，这两个是拆箱改装后的理论值。
// 扭矩会掉到 1/减速比：M3508 → 0.31 N·m 满量程，M2006 → 0.093 N·m 满量程。
inline constexpr djiMotorSpec kM3508Direct{
        "M3508/direct", DjiCtrlMode::Current,
        1.0f,                       // gear_ratio：直连
        16384.0f, 20.0f,            // 控制量程（配 C620）
        16384.0f, 20.0f,            // 反馈电流量程
        0.3f / 19.2032f,            // 手册输出轴 Kt(0.3) ÷ P19齿比(19.2032) = 转子侧 0.015622
        false
};

inline constexpr djiMotorSpec kM2006Direct{
        "M2006/direct", DjiCtrlMode::Current,
        1.0f,
        10000.0f, 10.0f,            // 控制量程（配 C610）
        10000.0f, 10.0f,
        0.3333333f / 36.0f,         // 手册输出轴 Kt(1N·m/3A=0.3333) ÷ P36齿比(36) = 转子侧 0.0092593
        false
};

inline constexpr djiMotorSpec kGM6020Direct{
        "GM6020/direct", DjiCtrlMode::Voltage,
        1.0f,                       // 本身无减速箱
        25000.0f, 24.0f,            // 电压控制
        16384.0f, 3.0f,             // 反馈电流 ±3 A
        1.2f / 1.62f,               // 手册额定点 1.2N·m / 1.62A，无减速箱 → 转子侧 0.74074
        true
};

// ── 装减速箱后的成品（最常用）──
inline constexpr djiMotorSpec kC620_M3508 =
        withGearRatio(kM3508Direct, 3591.0f / 187.0f, "M3508/C620");      // ≈ 19.2032
inline constexpr djiMotorSpec kC610_M2006 =
        withGearRatio(kM2006Direct, 36.0f, "M2006/C610");
inline constexpr djiMotorSpec kGM6020_Voltage = kGM6020Direct;

// GM6020 电流控制（同一台电机，换控制模式）
inline constexpr djiMotorSpec kGM6020_Current{
        "GM6020/current", DjiCtrlMode::Current,
        1.0f,
        16384.0f, 3.0f,             // 电流控制 ±3 A
        16384.0f, 3.0f,
        1.2f / 1.62f,
        true
};

// 拆箱直驱别名
inline constexpr djiMotorSpec kC620_M3508_Direct = kM3508Direct;
inline constexpr djiMotorSpec kC610_M2006_Direct = kM2006Direct;

// 例：M3508 输出轴再串一级 2:1 同步带 → 总减速比 19.2032 × 2
// inline constexpr djiMotorSpec kC620_M3508_x2 =
//         withExtraGear(kC620_M3508, 2.0f, "M3508/C620+x2");

class djiMotor : public MotorBase, public canNode
{
        public:
                static constexpr uint32_t kFirstFbId = 0x201U;   // 反馈ID 0x201~0x20B
                static constexpr uint32_t kGroupSize  = 4U;      // 一帧 8 字节带 4 台

                // tx_frame_id : 控制帧 ID，同组 4 台共享（0x200 / 0x1FF / 0x1FE / 0x2FF）
                // feedback_id : 本机反馈帧 ID（0x201~0x20B），决定它在帧里的槽位
                djiMotor(
                        CAN_HandleTypeDef  *hcan,
                        uint32_t            tx_frame_id,
                        uint32_t            feedback_id,
                        const djiMotorSpec &spec
                )
                        : MotorBase(spec.maxTorque()),
                          canNode(hcan, tx_frame_id, feedback_id, canCfg::kIdMaskNone),
                          spec_(spec) {}

                // 反馈帧解析（由 CAN 中断回调按 feedback_id 分发进来）
                HAL_StatusTypeDef onFrame(const uint8_t *data, uint16_t len) override;

                // 组初始化。参数可为 nullptr，顺序任意，内部校验同组/去重/按 ID 排序。
                static HAL_StatusTypeDef groupInit(
                        djiMotor *dji_motor1 = nullptr,
                        djiMotor *dji_motor2 = nullptr,
                        djiMotor *dji_motor3 = nullptr,
                        djiMotor *dji_motor4 = nullptr
                );

                HAL_StatusTypeDef sendTorque(float torque) override;

                // 整组打包成一帧，用本组共用帧 ID 发出。组内任意成员调用一次即可。
                HAL_StatusTypeDef groupSendTorque();

                const djiMotorSpec &spec() const { return spec_; }

                // 反馈量（已换算到输出轴）
                float getTorque() const { return torque_; }   // N·m
                float getSpeed() const { return speed_; }     // rpm（输出轴）
                float getAngle() const { return angle_; }     // 度（输出轴）
                float getTemp() const { return temp_; }       // ℃
                float getMaxTorque() const { return spec_.maxTorque(); }

        private:
                int16_t  torqueToRaw() const;
                uint8_t  slot() const;      // 本机在帧内的槽位 = (feedback_id-0x201)%4

                djiMotorSpec spec_ = {};          // 按值存：允许运行期自定义减速箱，也不会悬垂

                //电机组,都怪邪恶dji
                djiMotor *group_[kGroupSize] = {};

                float torque_ = 0.0f;
                float speed_ = 0.0f;
                float angle_ = 0.0f;
                float temp_ = 0.0f;
};

#endif //THREE_FRICTION_DRV_DJI_MOTOR_HPP
