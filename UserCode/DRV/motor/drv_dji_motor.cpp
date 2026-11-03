//
// Created by ye on 2026/10/30.
//

#include "drv_dji_motor.hpp"

#include <cstring>

#include "dwt.hpp"
#include "utils.hpp"

HAL_StatusTypeDef DjiMotor::OnFrame(const uint8_t* data, uint16_t len) {
    if (data == nullptr || len < 7U) {
        return HAL_ERROR;
    }

    const int16_t raw_angle = utils::BeI16(&data[0]);
    const int16_t raw_speed = utils::BeI16(&data[2]);
    const int16_t raw_current = utils::BeI16(&data[4]);

    angle_ = static_cast<float>(raw_angle) * spec_.AngleScale();
    speed_ = static_cast<float>(raw_speed) * spec_.SpeedScale();

    // raw → A → N·m（输出轴）
    const float cur_a = static_cast<float>(raw_current) * (spec_.fb_unit_full / spec_.fb_raw_full);
    torque_ = cur_a * spec_.KtOutput();

    temp_ = static_cast<float>(data[6]);

    UpdateTotalAngle(raw_angle);

    return HAL_OK;
}

HAL_StatusTypeDef DjiMotor::GroupInit(DjiMotor* dji_motor1, DjiMotor* dji_motor2,
                                      DjiMotor* dji_motor3, DjiMotor* dji_motor4) {
    DjiMotor* const motor[kGroupSize] = {dji_motor1, dji_motor2, dji_motor3, dji_motor4};
    if (motor[0] == nullptr) return HAL_ERROR;
    CAN_HandleTypeDef* group_hcan = motor[0]->Hcan();
    DjiMotor* group[kGroupSize] = {};

    for (uint8_t i = 0; i < kGroupSize; i++) {
        if (motor[i] == nullptr) continue;

        // ③ 必须同一条 CAN，且共用同一个控制帧 ID
        if (motor[i]->Hcan() != group_hcan) return HAL_ERROR;

        // ⑥ 按反馈 ID 升序：槽位 = (fb_id-0x201)%4，顺序错了电流会打到别的电机上
        group[motor[i]->Slot()] = motor[i];
    }

    // ⑦ 组内每个成员持有同一份组表，空槽为 nullptr
    for (uint8_t i = 0; i < kGroupSize; i++) {
        if (motor[i] == nullptr) continue;
        std::memcpy(motor[i]->group_, group, sizeof(group));
    }

    return HAL_OK;
}

motor::Status DjiMotor::SetTargetTorqueNm(float torque_nm) {
    if (!std::isfinite(torque_nm) || max_torque_nm_ <= 0.0f) return motor::Status::kInvalidArgument;
    target_torque_nm_ = std::clamp(torque_nm, -max_torque_nm_, max_torque_nm_);
    return motor::Status::kOk;
}

motor::Status DjiMotor::SendTorque() {
    if (Slot() == 0U) {
        return GroupSendTorque() == HAL_OK ? motor::Status::kOk : motor::Status::kIoError;
    }
    return motor::Status::kOk;
}

HAL_StatusTypeDef DjiMotor::GroupSendTorque() {
    uint8_t frame[8] = {};

    for (uint8_t i = 0; i < kGroupSize; i++) {
        DjiMotor* m = group_[i];
        if (m == nullptr) continue;

        utils::PutBeI16(&frame[2U * m->Slot()], m->torqueToRaw());
    }

    // 组内四台共用同一个控制帧 ID（canNode::Send 用的是 id_）
    return SendStandard(frame, sizeof(frame));
}

uint8_t DjiMotor::Slot() const {
    return static_cast<uint8_t>((FeedbackId() - kFirstFbId) % kGroupSize);
}

int16_t DjiMotor::torqueToRaw() const {
    // N·m → 协议 raw：满量程力矩对应 ±ctrl_raw_full
    const float ratio = std::clamp(target_torque_nm_ / spec_.MaxTorque(), -1.0f, 1.0f);
    return static_cast<int16_t>(std::lround(ratio * spec_.ctrl_raw_full));
}

void DjiMotor::UpdateTotalAngle(int16_t raw_angle) {
    // 判圈：编码器值跳变超过半圈，说明跨圈（两帧间转过半圈会漏判，见注释）
    if (total_angle_ready_) {
        if (raw_angle - last_raw_angle_ > kEncHalfTurn) {
            round_--;
        } else if (raw_angle - last_raw_angle_ < -kEncHalfTurn) {
            round_++;
        }
    }
    last_raw_angle_ = raw_angle;

    // 累计角度（度，输出轴）
    total_angle_ = 360.0f * (static_cast<float>(round_) + static_cast<float>(raw_angle) / 8192.0f) /
                   spec_.gear_ratio;

    UpdateTotalAngleSpeed();
}

void DjiMotor::UpdateTotalAngleSpeed(float k) {
    const float Dt = Dwt::DeltaT(dwt_cnt_);

    if (!total_angle_ready_) {
        // 第一帧只记基准：此时 Dt 是从 0 起算的，算出来的速度没意义
        total_angle_ready_ = true;
        total_angle_last_ = total_angle_;
        total_angle_speed_ = 0.0f;
        return;
    }

    if (Dt <= 0.0f) {
        return;
    }

    // 度/秒 → rpm，再做一阶低通
    float speed = (total_angle_ - total_angle_last_) / Dt * 60.0f / 360.0f;
    speed = k * speed + (1.0f - k) * total_angle_speed_;

    total_angle_speed_ = speed;
    total_angle_last_ = total_angle_;
}
