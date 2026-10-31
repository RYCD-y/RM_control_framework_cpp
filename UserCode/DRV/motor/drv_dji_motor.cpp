//
// Created by ye on 2026/10/30.
//

#include "drv_dji_motor.hpp"
#include <cstring>

HAL_StatusTypeDef djiMotor::onFrame(const uint8_t *data, uint16_t len)
{
        if (data == nullptr || len < 7U) { return HAL_ERROR; }

        const int16_t raw_angle   = utils::be_i16(&data[0]);
        const int16_t raw_speed   = utils::be_i16(&data[2]);
        const int16_t raw_current = utils::be_i16(&data[4]);

        angle_ = static_cast<float>(raw_angle) * spec_.angleScale();
        speed_ = static_cast<float>(raw_speed) * spec_.speedScale();

        // raw → A → N·m（输出轴）
        const float cur_a = static_cast<float>(raw_current) * (spec_.fb_unit_full / spec_.fb_raw_full);
        torque_           = cur_a * spec_.ktOutput();

        temp_ = static_cast<float>(data[6]);

        return HAL_OK;
}

HAL_StatusTypeDef djiMotor::groupInit(
        djiMotor *dji_motor1,
        djiMotor *dji_motor2,
        djiMotor *dji_motor3,
        djiMotor *dji_motor4
)
{
        djiMotor *const motor[kGroupSize] = {
                dji_motor1,
                dji_motor2,
                dji_motor3,
                dji_motor4
        };
        if (motor[0] == nullptr) return HAL_ERROR;
        CAN_HandleTypeDef *group_hcan        = motor[0]->getHcan();
        djiMotor *         group[kGroupSize] = {};

        for (uint8_t i = 0; i < kGroupSize; i++)
        {
                if (motor[i] == nullptr) continue;

                // ③ 必须同一条 CAN，且共用同一个控制帧 ID
                if (motor[i]->getHcan() != group_hcan) return HAL_ERROR;

                // ⑥ 按反馈 ID 升序：槽位 = (fb_id-0x201)%4，顺序错了电流会打到别的电机上
                group[motor[i]->slot()] = motor[i];
        }

        // ⑦ 组内每个成员持有同一份组表，空槽为 nullptr
        for (uint8_t i = 0; i < kGroupSize; i++)
        {
                if (motor[i] == nullptr) continue;
                std::memcpy(motor[i]->group_, group, sizeof(group));
        }


        return HAL_OK;
}

HAL_StatusTypeDef djiMotor::sendTorque(float torque)
{
        // (void) setTorque(torque);            // 记录本机目标力矩（已按 maxTorque 钳位）

        if (slot() == 0U) { return groupSendTorque(); } // 组内 0 号统一发一帧
        return HAL_OK;
}

HAL_StatusTypeDef djiMotor::groupSendTorque()
{
        uint8_t frame[8] = {};

        for (uint8_t i = 0; i < kGroupSize; i++)
        {
                djiMotor *m = group_[i];
                if (m == nullptr) continue;

                utils::put_be_i16(&frame[2U * m->slot()], m->torqueToRaw());
        }

        // 组内四台共用同一个控制帧 ID（canNode::send 用的是 id_）
        return send(frame, sizeof(frame));
}

uint8_t djiMotor::slot() const
{
        return static_cast<uint8_t>((getFeedbackId() - kFirstFbId) % kGroupSize);
}

int16_t djiMotor::torqueToRaw() const
{
        // N·m → 协议 raw：满量程力矩对应 ±ctrl_raw_full
        const float ratio = std::clamp(target_torque_ / spec_.maxTorque(), -1.0f, 1.0f);
        return static_cast<int16_t>(std::lround(ratio * spec_.ctrl_raw_full));
}
