//
// Created by ye on 2026/10/30.
//

#include "hal_can.hpp"

// 全局节点表定义（各节点构造时注册、析构时移除）
etl::vector<CanNode*, canCfg::kCanNodeSizeMax> CanNode::node_tab_;

// 发标准帧：ID 取低 11 位
HAL_StatusTypeDef CanNode::SendStandard(const uint8_t* data, uint16_t len) {
    if (hcan_ == nullptr || data == nullptr || len > 8U) {
        return HAL_ERROR;
    }

    CAN_TxHeaderTypeDef tx_header = {};
    tx_header.StdId = id_ & canCfg::kStdIdMask;
    tx_header.ExtId = 0x00;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0;
    // HAL 的 aData 形参是 uint8_t[]（非 const），只能就地去掉 const
    return HAL_CAN_AddTxMessage(hcan_, &tx_header, const_cast<uint8_t*>(data), &mailbox);
}

// 发扩展帧：ID 取低 29 位
HAL_StatusTypeDef CanNode::SendExtended(const uint8_t* data, uint16_t len) {
    if (hcan_ == nullptr || data == nullptr || len > 8U) {
        return HAL_ERROR;
    }

    CAN_TxHeaderTypeDef tx_header = {};
    tx_header.ExtId = id_ & canCfg::kExtIdMask;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0;
    return HAL_CAN_AddTxMessage(hcan_, &tx_header, const_cast<uint8_t*>(data), &mailbox);
}

// 按显式扩展 ID 发送：用于每条命令高字节不同的协议（如 VESC）
HAL_StatusTypeDef CanNode::SendExtendedWithId(uint32_t ext_id, const uint8_t* data,
                                              uint16_t len) {
    if (hcan_ == nullptr || data == nullptr || len > 8U) {
        return HAL_ERROR;
    }

    CAN_TxHeaderTypeDef tx_header = {};
    tx_header.ExtId = ext_id & canCfg::kExtIdMask;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0;
    return HAL_CAN_AddTxMessage(hcan_, &tx_header, const_cast<uint8_t*>(data), &mailbox);
}

// 基类默认不处理任何帧
HAL_StatusTypeDef CanNode::OnFrame(const uint8_t* data, uint16_t len) { return HAL_ERROR; }

// 配置过滤器（全放行 + FIFO0）并开启接收/错误通知
HAL_StatusTypeDef CanNode::FilterInit(CAN_HandleTypeDef* hcan) {
    if (hcan == nullptr) {
        return HAL_ERROR;
    }

    const uint32_t bank =
        (hcan->Instance == CAN2) ? canCfg::kCan2FilterBank : canCfg::kCan1FilterBank;

    CAN_FilterTypeDef sFilterConfig = {};
    sFilterConfig.FilterBank = bank;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;  // mask=0 → 全部通过(软件层按ID分发)
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    // 该字段写的是CAN1的FMR.CAN2SB(CAN2的过滤寄存器只能从CAN1侧访问), 两路必须填一致
    sFilterConfig.SlaveStartFilterBank = canCfg::kCan2FilterBank;

    const HAL_StatusTypeDef st = HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
    if (st != HAL_OK) {
        return st;
    }

    return HAL_CAN_ActivateNotification(
        hcan, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_WARNING |
                  CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_ERROR);
}

// ─────────────── CAN 中断回调（覆盖 HAL 的 weak 版本）───────────────
// CAN 接收中断：取出 FIFO0 所有帧，按 ID 分发给已注册节点
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CAN_RxHeaderTypeDef hdr = {};
    uint8_t data[8] = {};

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) {
            break;
        }

        const uint32_t id = (hdr.IDE == CAN_ID_EXT) ? hdr.ExtId : hdr.StdId;
        ++canCfg::g_rx_frame_count;

        for (CanNode* node : CanNode::Nodes()) {
            if (node == nullptr || node->Hcan() != hcan) {
                continue;
            }
            if ((id & node->IdMask()) == (node->FeedbackId() & node->IdMask())) {
                (void)node->OnFrame(data, static_cast<uint16_t>(hdr.DLC));
            }
        }
    }
}
