//
// 全局外设对象声明（只给 C++ 文件用）
// 定义在 EntMain.cpp，构造只绑定句柄不碰硬件，Init() 在 EntMain() 里调用。
//
#ifndef THREE_FRICTION_ENT_DEVICES_HPP
#define THREE_FRICTION_ENT_DEVICES_HPP

#include "buzzer/drv_buzzer.hpp"
#include "led/drv_led.hpp"

extern Led led;        // TIM5 CH1/CH2/CH3（ARR = 255）
extern Buzzer buzzer;  // TIM4 CH3

#endif  // THREE_FRICTION_ENT_DEVICES_HPP
