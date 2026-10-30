//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_HAL_CAN_HPP
#define THREE_FRICTION_HAL_CAN_HPP

#include <stdint.h>

#include "can.h"
#include "hal_bus.hpp"

class canNode : public BusNode
{
        public:
                //掩码预设
                static constexpr uint32_t IdMaskNone = 0xFFFF;  //精确匹配( 默认)
                static constexpr uint32_t IdMaskLow8 = 0xFF00;  //精确匹配( 默认)
                canNode(
                        CAN_HandleTypeDef *hcan,
                        uint32_t           id,
                        uint32_t           feedback_id,
                        uint32_t           id_mask
                )
                        : BusNode(id, feedback_id, id_mask),
                          hcan_(hcan) {}

                HAL_StatusTypeDef send(const uint8_t *data, uint16_t len) override;

                HAL_StatusTypeDef sendEx(const uint8_t *data, uint16_t len);

                //CAN过滤器初始化: CAN1→bank0, CAN2→bank14, 全部放行(标准帧+扩展帧)→FIFO0
                static void filterInit(CAN_HandleTypeDef *hcan);


        protected:
                CAN_HandleTypeDef *hcan_ = nullptr;
};

#endif //THREE_FRICTION_HAL_CAN_HPP
