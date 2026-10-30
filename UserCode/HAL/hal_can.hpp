//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_HAL_CAN_HPP
#define THREE_FRICTION_HAL_CAN_HPP

#include <stdint.h>

#include "can.h"
#include "hal_bus.hpp"

class CanNode : public BusNode
{
        public:
                CanNode(
                        CAN_HandleTypeDef *hcan,
                        uint32_t           id,
                        uint32_t           feedback_id,
                        uint32_t           id_mask
                )
                        : BusNode(id, feedback_id, id_mask),
                          hcan_(hcan) {}

                HAL_StatusTypeDef send(const uint8_t *data, uint16_t len) override;

                HAL_StatusTypeDef sendEx(const uint8_t *data, uint16_t len)
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

                HAL_StatusTypeDef onFrame(const uint8_t *data, uint16_t len) override
                {
                        (void) data;
                        (void) len;
                        return HAL_OK;
                }

        protected:
                CAN_HandleTypeDef *hcan_ = nullptr;
};

#endif //THREE_FRICTION_HAL_CAN_HPP
