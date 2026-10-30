//
// Created by ye on 2026/10/30.
//

#include "hal_can.hpp"

HAL_StatusTypeDef CanNode::send(const uint8_t *data, uint16_t len)
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
