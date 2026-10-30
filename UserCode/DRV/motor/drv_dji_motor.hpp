//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_DRV_DJI_MOTOR_HPP
#define THREE_FRICTION_DRV_DJI_MOTOR_HPP

#include "hal_can.hpp"
#include "motor_base.hpp"
#include "utils.hpp"

struct djiMotorSpec {
        const char* name;
        float       raw_full_scale;   // 协议满量程：16384 / 10000 / 25000
        float       unit_full_scale;  // 对应物理量满量程：20A / 10A / 24V
        float       torque_per_unit;  // 电流(A)或电压(V) → 输出力矩(N·m)，纯标定系数
        float       speed_scale;      // 反馈转速 LSB → rad/s
        float       angle_scale;      // 反馈角度 LSB → rad
        bool        angle_is_absolute;// GM6020 是绝对角，M3508 是靠圈数累加
};

inline constexpr djiMotorSpec kC620_M3508 {
        "M3508/C620", 16384.0f, 20.0f,
        0.3f * 0.0185f,   // 0.3 N·m/A ÷ 3591/187 减速比  → 按你的实际减速比改
        0.01938f,         // (2π/60) / 3591*187  → RPM 转 rad/s
        2.0f * 3.14159265f / 8192.0f,
        false
};

inline constexpr djiMotorSpec kC610_M2006 {
        "M2006/C610", 10000.0f, 10.0f,
        0.18f * 0.0444f,  // 0.18 N·m/A ÷ 36 减速比
        0.01938f,
        2.0f * 3.14159265f / 8192.0f,
        false
};

inline constexpr djiMotorSpec kGM6020 {
        "GM6020", 25000.0f, 24.0f,
        1.0f / 0.741f,    // V → N·m
        0.01938f,
        2.0f * 3.14159265f / 8192.0f,
        true
};

class djiMotor : public MotorBase, public canNode
{
        public:
                djiMotor(
                        CAN_HandleTypeDef *hcan,
                        uint32_t           id,
                        uint32_t           feedback_id,

                        float              max_torque
                )
                        : MotorBase(max_torque),
                          canNode(
                                  hcan,
                                  id,
                                  feedback_id,
                                  IdMaskNone
                                 )
                {}

                HAL_StatusTypeDef sendTorque(float torque) override;

                HAL_StatusTypeDef groupInit(
                        djiMotor *dji_motor1 = nullptr,
                        djiMotor *dji_motor2 = nullptr,
                        djiMotor *dji_motor3 = nullptr,
                        djiMotor *dji_motor4 = nullptr
                );

        private:
                //电机组,都怪邪恶dji
                djiMotor *group[4] = {};
};

#endif //THREE_FRICTION_DRV_DJI_MOTOR_HPP
