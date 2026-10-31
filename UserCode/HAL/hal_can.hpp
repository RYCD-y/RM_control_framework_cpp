//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_HAL_CAN_HPP
#define THREE_FRICTION_HAL_CAN_HPP

#include <stdint.h>

#include "can.h"
#include "hal_bus.hpp"

namespace can_cfg
{
        // STM32F407 双 bxCAN 共享 28 个过滤器组：CAN1 用 0~13，CAN2 用 14~27
        inline constexpr uint32_t kTotalFilterBanks = 28U;
        inline constexpr uint32_t kCan1FilterBank   = 0U;
        inline constexpr uint32_t kCan2FilterBank   = kTotalFilterBanks / 2U;   // 14

        inline constexpr uint32_t kStdIdMask = 0x7FFU;
        inline constexpr uint32_t kExtIdMask = 0x1FFFFFFFU;

        //掩码预设
        static constexpr uint32_t kIdMaskNone = 0x7FF;   //标准帧11位全匹配
        static constexpr uint32_t kIdMaskLow8 = 0x780;   //只比较高4位(ID 分组)
} // namespace can_cfg

class canNode : public BusNode
{
        public:
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

                //CAN过滤器初始化: CAN1→bank0, CAN2→bank14, 全部放行→FIFO0
                static HAL_StatusTypeDef filterInit(CAN_HandleTypeDef *hcan);

                CAN_HandleTypeDef *getHcan() const { return hcan_; }

        protected:
                CAN_HandleTypeDef *hcan_ = nullptr;
};

#endif //THREE_FRICTION_HAL_CAN_HPP
