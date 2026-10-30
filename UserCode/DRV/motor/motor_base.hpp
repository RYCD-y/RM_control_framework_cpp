//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_MOTOR_BASE_HPP
#define THREE_FRICTION_MOTOR_BASE_HPP

class Motor
{
        protected:
                float target_torque;

                float torque;
                float speed;
                float angle;

                float temp;

        public:
                void setTorque(const float tor)
                {
                        target_torque = tor;
                }
};

#endif //THREE_FRICTION_MOTOR_BASE_HPP
