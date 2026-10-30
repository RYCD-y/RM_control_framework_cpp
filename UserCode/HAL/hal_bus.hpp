//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_HAL_BUS_HPP
#define THREE_FRICTION_HAL_BUS_HPP

#include <stdint.h>

// 只用官方入口头。直接 include "stm32f4xx_hal_def.h" 会踩包含顺序陷阱（见说明）。
#include "stm32f4xx_hal.h"

class BusNode
{
        public:
                virtual ~BusNode() = default;

                virtual HAL_StatusTypeDef send(const uint8_t* data, uint16_t len)     = 0;
                virtual HAL_StatusTypeDef onFrame(const uint8_t* data, uint16_t len)  = 0;

                BusNode(uint32_t id, uint32_t feedback_id, uint32_t id_mask)
                        : id_(id), feedback_id_(feedback_id), id_mask_(id_mask) {}

                uint32_t setId(uint32_t id)
                {
                        id_ = id;
                        return id_;
                }

                uint32_t setFeedbackId(uint32_t feedback_id)
                {
                        feedback_id_ = feedback_id;
                        return feedback_id_;
                }

                uint32_t setIdMask(uint32_t id_mask)
                {
                        id_mask_ = id_mask;
                        return id_mask_;
                }

                uint32_t getId() const { return id_; }
                uint32_t getFeedbackId() const { return feedback_id_; }
                uint32_t getIdMask() const { return id_mask_; }

        protected:
                uint32_t id_          = 0x00;
                uint32_t feedback_id_ = 0x00;
                uint32_t id_mask_     = 0x00;
};

#endif //THREE_FRICTION_HAL_BUS_HPP
