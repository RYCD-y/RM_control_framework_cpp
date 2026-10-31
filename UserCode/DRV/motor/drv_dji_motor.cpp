//
// Created by ye on 2026/10/30.
//

#include "drv_dji_motor.hpp"

// ─────────────── 反馈分发（CAN 中断回调 → 各节点 onFrame）───────────────
namespace
{
        constexpr uint8_t kMaxRegisteredNodes = 16U;

        canNode *s_nodes[kMaxRegisteredNodes] = {};
        uint8_t  s_node_count                 = 0U;

        void registerNode(canNode *node)
        {
                if (node == nullptr) { return; }
                for (uint8_t i = 0; i < s_node_count; i++)
                {
                        if (s_nodes[i] == node) { return; }
                }
                if (s_node_count < kMaxRegisteredNodes) { s_nodes[s_node_count++] = node; }
        }
} // namespace

HAL_StatusTypeDef djiMotor::onFrame(const uint8_t *data, uint16_t len)
{
        if (data == nullptr || len < 7U) { return HAL_ERROR; }

        const int16_t raw_angle = utils::be_i16(&data[0]);
        const int16_t raw_speed = utils::be_i16(&data[2]);
        const int16_t raw_cur   = utils::be_i16(&data[4]);

        angle_ = static_cast<float>(raw_angle) * spec_.angleScale();
        speed_ = static_cast<float>(raw_speed) * spec_.speedScale();

        // raw → A → N·m（输出轴）
        const float cur_a = static_cast<float>(raw_cur) * (spec_.fb_unit_full / spec_.fb_raw_full);
        torque_           = cur_a * spec_.ktOutput();

        temp_ = static_cast<float>(data[6]);

        return HAL_OK;
}

HAL_StatusTypeDef djiMotor::groupInit(djiMotor *dji_motor1,
                                      djiMotor *dji_motor2,
                                      djiMotor *dji_motor3,
                                      djiMotor *dji_motor4)
{
        djiMotor *const in[kGroupSize] = {dji_motor1, dji_motor2, dji_motor3, dji_motor4};

        djiMotor *members[kGroupSize] = {};
        uint8_t   count               = 0;
        uint32_t  group_idx           = 0;
        bool      group_set           = false;

        for (uint8_t i = 0; i < kGroupSize; i++)
        {
                djiMotor *m = in[i];
                if (m == nullptr) { continue; }

                // ① 同一对象不能出现两次
                for (uint8_t j = 0; j < count; j++)
                {
                        if (members[j] == m) { return HAL_ERROR; }
                }

                // ② 反馈 ID 必须在 0x201~0x20B
                if (m->getFeedbackId() < kFirstFbId ||
                    m->getFeedbackId() >= kFirstFbId + 3U * kGroupSize)
                {
                        return HAL_ERROR;
                }

                // ③ 必须同一条 CAN，且共用同一个控制帧 ID
                if (m->hcan_ != members[0]->hcan_) { return HAL_ERROR; }
                if (m->getId() != members[0]->getId()) { return HAL_ERROR; }

                // ④ 必须同一组：0x201~0x204、0x205~0x208、0x209~0x20B
                const uint32_t idx = (m->getFeedbackId() - kFirstFbId) / kGroupSize;
                if (!group_set)
                {
                        group_idx = idx;
                        group_set = true;
                }
                else if (idx != group_idx)
                {
                        return HAL_ERROR;
                }

                // ⑤ 同组必须同控制模式（一帧里量纲必须一致）
                if (m->spec_.mode != members[0]->spec_.mode) { return HAL_ERROR; }

                members[count] = m;
                count++;
        }

        if (count == 0U) { return HAL_ERROR; }

        // ⑥ 按反馈 ID 升序：槽位 = (fb_id-0x201)%4，顺序错了电流会打到别的电机上
        for (uint8_t i = 1; i < count; i++)
        {
                djiMotor *key = members[i];
                uint8_t   j   = i;
                while (j > 0U && members[j - 1U]->getFeedbackId() > key->getFeedbackId())
                {
                        members[j] = members[j - 1U];
                        j--;
                }
                members[j] = key;
        }

        // ⑦ 组内每个成员持有同一份组表，空槽为 nullptr
        for (uint8_t i = 0; i < count; i++)
        {
                for (uint8_t k = 0; k < kGroupSize; k++)
                {
                        members[i]->group_[k] = (k < count) ? members[k] : nullptr;
                }
                registerNode(members[i]); // 注册进中断分发表
        }

        return HAL_OK;
}

HAL_StatusTypeDef djiMotor::groupSend()
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
        const float max_nm = spec_.maxTorque();
        if (max_nm <= 0.0f) { return 0; }

        const float ratio = std::clamp(target_torque_ / max_nm, -1.0f, 1.0f);
        return static_cast<int16_t>(std::lround(ratio * spec_.ctrl_raw_full));
}

// ─────────────── CAN 中断回调（覆盖 HAL 的 weak 版本）───────────────
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
        CAN_RxHeaderTypeDef hdr     = {};
        uint8_t             data[8] = {};

        while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
        {
                if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) { break; }

                const uint32_t id = (hdr.IDE == CAN_ID_EXT) ? hdr.ExtId : hdr.StdId;

                for (uint8_t i = 0; i < s_node_count; i++)
                {
                        canNode *n = s_nodes[i];
                        if (n == nullptr || n->getHcan() != hcan) { continue; }
                        if ((id & n->getIdMask()) == (n->getFeedbackId() & n->getIdMask()))
                        {
                                (void) n->onFrame(data, static_cast<uint16_t>(hdr.DLC));
                        }
                }
        }
}
