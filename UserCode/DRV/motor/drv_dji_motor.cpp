//
// Created by ye on 2026/10/30.
//

#include "drv_dji_motor.hpp"

HAL_StatusTypeDef djiMotor::sendTorque(float torque)
{
        uint8_t Motor_Tx_Data[8];
        Motor_Tx_Data[0] = utils::float_to_signed<int16_t>(group[0]->target_torque_, max_torque_) >> 8;
        Motor_Tx_Data[1] = utils::float_to_signed<int16_t>(group[0]->target_torque_, max_torque_)  & 0xFF;
        Motor_Tx_Data[2] = utils::float_to_signed<int16_t>(group[1]->target_torque_, max_torque_)  >> 8;
        Motor_Tx_Data[3] = utils::float_to_signed<int16_t>(group[1]->target_torque_, max_torque_)  & 0xFF;
        Motor_Tx_Data[4] = utils::float_to_signed<int16_t>(group[2]->target_torque_, max_torque_)  >> 8;
        Motor_Tx_Data[5] = utils::float_to_signed<int16_t>(group[2]->target_torque_, max_torque_)  & 0xFF;
        Motor_Tx_Data[6] = utils::float_to_signed<int16_t>(group[3]->target_torque_, max_torque_)  >> 8;
        Motor_Tx_Data[7] = utils::float_to_signed<int16_t>(group[3]->target_torque_, max_torque_)  & 0xFF;

        return canNode::send(Motor_Tx_Data, sizeof(Motor_Tx_Data));
}

HAL_StatusTypeDef djiMotor::groupInit(
        djiMotor *dji_motor1,
        djiMotor *dji_motor2,
        djiMotor *dji_motor3,
        djiMotor *dji_motor4
        )
{
        djiMotor *dji_motor[4] = {
                dji_motor1,
                dji_motor2,
                dji_motor3,
                dji_motor4
        };
        for (uint8_t i = 0; i < 4; i++)
        {
                if (dji_motor[i] == nullptr) continue;
                for (uint8_t j = 0; j < 4; j++)
                        dji_motor[i]->group[j] = dji_motor[j];
        }
        return HAL_OK;
}
