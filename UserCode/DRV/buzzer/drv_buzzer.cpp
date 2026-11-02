//
// 蜂鸣器驱动实现（音调 = 改 PSC/ARR 产生目标频率，音效 = 一串音符 + 延时）
//
#include "buzzer/drv_buzzer.hpp"

namespace
{
        // 音调频率表（Hz），索引与 Buzzer::Tone 一一对应；Pause 用 0 占位
        constexpr uint16_t kToneFreqHz[] = {
                0,                                        // Pause
                262, 294, 330, 349, 392, 440, 494,        // L1 ~ L7
                523, 587, 659, 698, 784, 880, 988,        // M1 ~ M7
                1046, 1175, 1318, 1397, 1568, 1760, 1976, // H1 ~ H7
        };
        constexpr uint8_t kToneCount = sizeof(kToneFreqHz) / sizeof(kToneFreqHz[0]);

        // 休止符（P）用很小的占空比，避免蜂鸣器持续响
        constexpr uint32_t kPausePeriod = 100;
} // namespace

Buzzer::Buzzer(TIM_HandleTypeDef *htim, uint32_t channel, TickType_t task_tick)
        : htim_(htim),
          channel_(channel),
          task_tick_(task_tick)
{}

/* ═══════════════════════ 基础控制 ═══════════════════════ */
bool Buzzer::init(const char *name)
{
        if (htim_ == nullptr) { return pdFAIL; }

        (void) HAL_TIM_Base_Start(htim_);
        (void) HAL_TIM_PWM_Start(htim_, channel_);

        return xTaskCreate(&Buzzer::taskEntry, name, kTaskStack, this, kTaskPriority, nullptr);
}

void Buzzer::start()
{
        if (htim_ == nullptr) { return; }
        (void) HAL_TIM_PWM_Start(htim_, channel_);
}

void Buzzer::stop()
{
        if (htim_ == nullptr) { return; }
        (void) HAL_TIM_PWM_Stop(htim_, channel_);
}

void Buzzer::setSoundEffect(SoundEffect effect)
{
        sound_effect_ = effect;
}

void Buzzer::setTone(Tone tone)
{
        if (htim_ == nullptr) { return; }

        const uint8_t index = static_cast<uint8_t>(tone);
        if (index >= kToneCount) { return; }

        const uint16_t psc = kBasePrescaler;
        uint32_t       arr = kPausePeriod;

        if (tone != Tone::Pause)
        {
                const uint32_t sys_freq = HAL_RCC_GetSysClockFreq();
                arr                     = sys_freq / psc / kToneFreqHz[index];
                if (arr == 0U) { arr = 1U; }
        }

        __HAL_TIM_SET_PRESCALER(htim_, psc - 1U);
        __HAL_TIM_SET_AUTORELOAD(htim_, arr - 1U);
        __HAL_TIM_SetCompare(htim_, channel_, arr / 2U);
}

/* ═══════════════════════ 任务 ═══════════════════════ */
void Buzzer::taskEntry(void *argument)
{
        static_cast<Buzzer *>(argument)->run();
}


void Buzzer::run()
{
        for (;;)
        {
                playSoundEffect();
                vTaskDelay(task_tick_);
        }
}

void Buzzer::playSoundEffect()
{
        const SoundEffect effect = sound_effect_; // 一次快照，避免播放中途被改写导致分支错乱

        switch (effect)
        {
        case SoundEffect::Off :
                setTone(Tone::Pause);
                sound_effect_ = SoundEffect::Off;
                stop();
                break;

        case SoundEffect::SystemStart :
                start();
                setTone(Tone::M1);
                vTaskDelay(200);
                setTone(Tone::Pause);
                vTaskDelay(50);
                setTone(Tone::M2);
                vTaskDelay(200);
                setTone(Tone::Pause);
                vTaskDelay(50);
                setTone(Tone::M3);
                vTaskDelay(200);
                setTone(Tone::Pause);
                vTaskDelay(50);
                setTone(Tone::M2);
                vTaskDelay(200);
                setTone(Tone::Pause);
                vTaskDelay(50);
                setTone(Tone::M5);
                vTaskDelay(200);
                setTone(Tone::Pause);
                vTaskDelay(50);
                setTone(Tone::M7);
                vTaskDelay(200);
                setTone(Tone::Pause);
                vTaskDelay(50);
                setTone(Tone::M6);
                vTaskDelay(500);
                setTone(Tone::M5);
                vTaskDelay(250);
                setTone(Tone::M6);
                vTaskDelay(750);
                sound_effect_ = SoundEffect::Off;
                break;

        case SoundEffect::Error :
                start();
                for (uint8_t i = 0; i < 3; i++)
                {
                        setTone(Tone::M7);
                        vTaskDelay(100);
                        setTone(Tone::Pause);
                        vTaskDelay(100);
                }
                sound_effect_ = SoundEffect::Off;
                break;

        case SoundEffect::SuperCapOn :
                start();
                setTone(Tone::M1);
                vTaskDelay(200);
                setTone(Tone::M3);
                vTaskDelay(200);
                setTone(Tone::M5);
                vTaskDelay(200);
                sound_effect_ = SoundEffect::Off;
                break;

        case SoundEffect::SuperCapOff :
                start();
                setTone(Tone::M5);
                vTaskDelay(200);
                setTone(Tone::M3);
                vTaskDelay(200);
                setTone(Tone::M1);
                vTaskDelay(200);
                sound_effect_ = SoundEffect::Off;
                break;

        case SoundEffect::AimOn :
                start();
                setTone(Tone::M7);
                vTaskDelay(50);
                setTone(Tone::Pause);
                vTaskDelay(150);
                setTone(Tone::M7);
                vTaskDelay(50);
                setTone(Tone::Pause);
                vTaskDelay(150);
                setTone(Tone::M7);
                vTaskDelay(50);
                setTone(Tone::Pause);
                vTaskDelay(150);
                sound_effect_ = SoundEffect::Off;
                break;

        case SoundEffect::AimOff :
                start();
                setTone(Tone::M7);
                vTaskDelay(200);
                setTone(Tone::M5);
                vTaskDelay(200);
                setTone(Tone::M3);
                vTaskDelay(200);
                sound_effect_ = SoundEffect::Off;
                break;

        // v2 里 Speed1~Speed5 有枚举但没有播放内容，这里保持"立即熄灭"
        case SoundEffect::Speed1 :
        case SoundEffect::Speed2 :
        case SoundEffect::Speed3 :
        case SoundEffect::Speed4 :
        case SoundEffect::Speed5 :
                sound_effect_ = SoundEffect::Off;
                break;

        case SoundEffect::Shoot1 :
        case SoundEffect::Shoot2 :
        case SoundEffect::Shoot3 :
        case SoundEffect::Shoot4 :
        case SoundEffect::Shoot5 :
        case SoundEffect::Shoot6 :
        case SoundEffect::Shoot7 :
        case SoundEffect::Shoot8 :
        {
                // ShootN = N 声 "M5 + 休止"，每声 100 ms
                const uint8_t shots = static_cast<uint8_t>(effect) -
                                      static_cast<uint8_t>(SoundEffect::Shoot1) + 1U;
                start();
                for (uint8_t i = 0; i < shots; i++)
                {
                        setTone(Tone::M5);
                        vTaskDelay(100);
                        setTone(Tone::Pause);
                        vTaskDelay(100);
                }
                sound_effect_ = SoundEffect::Off;
                break;
        }
        }
}
