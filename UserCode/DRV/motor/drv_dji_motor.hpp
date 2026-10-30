//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_DRV_DJI_MOTOR_HPP
#define THREE_FRICTION_DRV_DJI_MOTOR_HPP

#include "hal_can.hpp"
#include "motor_base.hpp"

class djiMotor : public MotorBase, public CanNode
{
        public:
                djiMotor(
                        CAN_HandleTypeDef *hcan,
                        uint32_t           id,
                        uint32_t           feedback_id,
                        uint32_t           id_mask,

                        float              max_torque
                )
                        : MotorBase(max_torque),
                          CanNode(
                                  hcan,
                                  id,
                                  feedback_id,
                                  id_mask
                                 )
                {}

                HAL_StatusTypeDef sendTorque(float torque) override;

                HAL_StatusTypeDef groupInit(
                        djiMotor *dji_motor1,
                        djiMotor *dji_motor2,
                        djiMotor *dji_motor3,
                        djiMotor *dji_motor4
                );

        private:
                //电机组,都怪邪恶dji
                djiMotor *group[4];
};

#endif //THREE_FRICTION_DRV_DJI_MOTOR_HPP
