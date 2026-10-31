//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_MOTOR_BASE_HPP
#define THREE_FRICTION_MOTOR_BASE_HPP

#include <algorithm>
#include "hal_bus.hpp"
#include "stm32f4xx_hal_def.h"

class MotorBase
{
        public:
                MotorBase(
                        float max_torque
                )
                        : max_torque_(max_torque)
                {}

                HAL_StatusTypeDef setTorque(float torque)
                {
                        target_torque_ = std::clamp(torque, -max_torque_, max_torque_);
                        return HAL_OK;
                }

                float getTargetTorque() const
                {
                        return target_torque_;
                }

                virtual HAL_StatusTypeDef sendTorque(float torque) = 0;

        protected:
                float target_torque_ = 0.0f;
                float max_torque_    = 0.0f;
};

#endif //THREE_FRICTION_MOTOR_BASE_HPP
