//
// Created by ye on 2026/10/30.
//

#include "hal_can.hpp"

HAL_StatusTypeDef canNode::send(const uint8_t *data, uint16_t len)
{
        if (hcan_ == nullptr || data == nullptr) { return HAL_ERROR; }
        if (len > 8U) { len = 8U; } // 经典 CAN 单帧上限

        CAN_TxHeaderTypeDef tx_header = {};
        tx_header.StdId               = id_;
        tx_header.ExtId               = 0x00;
        tx_header.IDE                 = CAN_ID_STD;
        tx_header.RTR                 = CAN_RTR_DATA;
        tx_header.DLC                 = len;
        tx_header.TransmitGlobalTime  = DISABLE;

        uint32_t mailbox = 0;
        // HAL 的 aData 形参是 uint8_t[]（非 const），只能就地去掉 const
        return HAL_CAN_AddTxMessage(
                                    hcan_,
                                    &tx_header,
                                    const_cast<uint8_t *>(data),
                                    &mailbox
                                   );
}

HAL_StatusTypeDef canNode::sendEx(const uint8_t *data, uint16_t len)
{
        if (hcan_ == nullptr || data == nullptr) { return HAL_ERROR; }
        if (len > 8) len = 8; //经典帧最大8字节, 只取有效长度, 其余保持0(HAL发送时仍会读满8字节缓冲区)

        CAN_TxHeaderTypeDef tx_header = {};
        tx_header.ExtId               = id_;
        tx_header.IDE                 = CAN_ID_EXT;
        tx_header.RTR                 = CAN_RTR_DATA;
        tx_header.DLC                 = len; //经典帧 0~8 字节时 DLC 即字节数
        tx_header.TransmitGlobalTime  = DISABLE;

        uint32_t mailbox = 0;
        // HAL 的 aData 形参是 uint8_t[]（非 const），只能就地去掉 const
        return HAL_CAN_AddTxMessage(
                                    hcan_,
                                    &tx_header,
                                    const_cast<uint8_t *>(data),
                                    &mailbox
                                   );
}

//STM32F407 双bxCAN共享28个过滤bank: CAN1用0~13, CAN2用14~27
inline constexpr uint32_t CAN2_START_FILTER_BANK = 28U / 2U;
void CAN_Filter_Init(CAN_HandleTypeDef *hcan)
{
        CAN_FilterTypeDef sFilterConfig = {0};

        sFilterConfig.FilterIdHigh         = 0x0000;
        sFilterConfig.FilterIdLow          = 0x0000;
        sFilterConfig.FilterMaskIdHigh     = 0x0000; //mask=0 → 全部通过(软件层按ID分发)
        sFilterConfig.FilterMaskIdLow      = 0x0000;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterBank           = (hcan->Instance == CAN2) ? CAN2_START_FILTER_BANK : 0;
        sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
        sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
        sFilterConfig.FilterActivation     = ENABLE;
        //该字段实际写的是CAN1的FMR.CAN2SB(CAN2的过滤寄存器只能从CAN1侧访问), 两路必须填一致
        sFilterConfig.SlaveStartFilterBank = CAN2_START_FILTER_BANK;

        HAL_CAN_ConfigFilter(hcan, &sFilterConfig);

        //中断
        HAL_CAN_ActivateNotification(hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_ERROR_WARNING |
                                     CAN_IT_ERROR_PASSIVE |
                                     CAN_IT_BUSOFF |
                                     CAN_IT_ERROR);
}
