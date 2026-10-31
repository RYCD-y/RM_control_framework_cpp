//
// Created by ye on 2026/10/30.
//

#include "hal_can.hpp"

HAL_StatusTypeDef canNode::send(const uint8_t *data, uint16_t len)
{
        if (hcan_ == nullptr || data == nullptr) { return HAL_ERROR; }
        if (len > 8U) { len = 8U; } // 经典 CAN 单帧上限

        CAN_TxHeaderTypeDef tx_header = {};
        tx_header.StdId               = id_ & can_cfg::kStdIdMask;
        tx_header.ExtId               = 0x00;
        tx_header.IDE                 = CAN_ID_STD;
        tx_header.RTR                 = CAN_RTR_DATA;
        tx_header.DLC                 = len;
        tx_header.TransmitGlobalTime  = DISABLE;

        uint32_t mailbox = 0;
        // HAL 的 aData 形参是 uint8_t[]（非 const），只能就地去掉 const
        return HAL_CAN_AddTxMessage(hcan_, &tx_header, const_cast<uint8_t *>(data), &mailbox);
}

HAL_StatusTypeDef canNode::sendEx(const uint8_t *data, uint16_t len)
{
        if (hcan_ == nullptr || data == nullptr) { return HAL_ERROR; }
        if (len > 8U) { len = 8U; }

        CAN_TxHeaderTypeDef tx_header = {};
        tx_header.ExtId              = id_ & can_cfg::kExtIdMask;
        tx_header.IDE                = CAN_ID_EXT;
        tx_header.RTR                = CAN_RTR_DATA;
        tx_header.DLC                = len;
        tx_header.TransmitGlobalTime = DISABLE;

        uint32_t mailbox = 0;
        return HAL_CAN_AddTxMessage(hcan_, &tx_header, const_cast<uint8_t *>(data), &mailbox);
}

HAL_StatusTypeDef canNode::filterInit(CAN_HandleTypeDef *hcan)
{
        if (hcan == nullptr) { return HAL_ERROR; }

        const uint32_t bank = (hcan->Instance == CAN2) ? can_cfg::kCan2FilterBank
                                                       : can_cfg::kCan1FilterBank;

        CAN_FilterTypeDef sFilterConfig = {};
        sFilterConfig.FilterBank           = bank;
        sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
        sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
        sFilterConfig.FilterIdHigh         = 0x0000;
        sFilterConfig.FilterIdLow          = 0x0000;
        sFilterConfig.FilterMaskIdHigh     = 0x0000; //mask=0 → 全部通过(软件层按ID分发)
        sFilterConfig.FilterMaskIdLow      = 0x0000;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterActivation     = ENABLE;
        //该字段写的是CAN1的FMR.CAN2SB(CAN2的过滤寄存器只能从CAN1侧访问), 两路必须填一致
        sFilterConfig.SlaveStartFilterBank = can_cfg::kCan2FilterBank;

        const HAL_StatusTypeDef st = HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
        if (st != HAL_OK) { return st; }

        return HAL_CAN_ActivateNotification(hcan,
                                            CAN_IT_RX_FIFO0_MSG_PENDING |
                                            CAN_IT_ERROR_WARNING |
                                            CAN_IT_ERROR_PASSIVE |
                                            CAN_IT_BUSOFF |
                                            CAN_IT_ERROR);
}
