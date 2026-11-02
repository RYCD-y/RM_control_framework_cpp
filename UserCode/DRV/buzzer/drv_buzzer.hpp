//
// 蜂鸣器（PWM 无源蜂鸣器）驱动 + 预设音效播放任务
// 对应 control_framework_v2/UserCode/DRV/buzzer/drv_buzzer.{h,c}，改为 C++ 类。
//
#ifndef THREE_FRICTION_DRV_BUZZER_HPP
#define THREE_FRICTION_DRV_BUZZER_HPP

#include <stdint.h>
#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "task.h"

class Buzzer
{
        public:
                // ── 任务参数 ──
                static constexpr uint16_t    kTaskStack    = 256; // words
                static constexpr UBaseType_t kTaskPriority = 2;

                /// 预设音效
                enum class SoundEffect : uint8_t
                {
                        Off = 0,
                        SystemStart,
                        Error,
                        SuperCapOn,
                        SuperCapOff,
                        AimOn,
                        AimOff,
                        Speed1, // 注：v2 里定义了但没有播放内容，这里沿用"立即熄灭"
                        Speed2,
                        Speed3,
                        Speed4,
                        Speed5,
                        Shoot1,
                        Shoot2,
                        Shoot3,
                        Shoot4,
                        Shoot5,
                        Shoot6,
                        Shoot7,
                        Shoot8,
                };

                /// 音调（P = 休止符）
                enum class Tone : uint8_t
                {
                        Pause = 0,
                        L1, L2, L3, L4, L5, L6, L7,
                        M1, M2, M3, M4, M5, M6, M7,
                        H1, H2, H3, H4, H5, H6, H7,
                };

                static constexpr TickType_t kDefaultTaskTick = 10; // ms，建议不超过 30
                static constexpr uint16_t   kBasePrescaler   = 50;

                /// 构造只绑定外设，不碰硬件（安全：可全局/静态定义）
                Buzzer(TIM_HandleTypeDef *htim, uint32_t channel,
                       TickType_t         task_tick = kDefaultTaskTick);

                bool init(const char *name);  // 启动基础定时器 + PWM（须在 MX_TIMx_Init 之后）
                void start(); // 打开 PWM 输出
                void stop();  // 关闭 PWM 输出

                void setSoundEffect(SoundEffect effect); // 供任意任务调用（含 ISR 之外的场景）
                void setTone(Tone tone);                 // 立即切换音调

                TickType_t taskTick() const { return task_tick_; }

                static void taskEntry(void *argument); // xTaskCreate(Buzzer::taskEntry, ..., this, ...)
                void        run();                     // 任务主体

        private:
                void playSoundEffect(); // 一次循环：按 sound_effect_ 播放

                TIM_HandleTypeDef *htim_      = nullptr;
                uint32_t           channel_   = 0;
                TickType_t         task_tick_ = kDefaultTaskTick;

                volatile SoundEffect sound_effect_ = SoundEffect::Off;
};


#endif //THREE_FRICTION_DRV_BUZZER_HPP
