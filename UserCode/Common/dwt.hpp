//
// DWT 周期计数器（微秒级时间基准）
// 对应 control_framework_v2/UserCode/HAL/hal_dwt.c，C → C++ 封装为全静态类。
//
#ifndef THREE_FRICTION_DWT_HPP
#define THREE_FRICTION_DWT_HPP

#include <stdint.h>

class Dwt
{
        public:
                static void     init(uint32_t cpu_freq_mhz);
                static float    deltaT(uint32_t &cnt_last);      // 上一次调用至今的时间（秒）
                static double   deltaT64(uint32_t &cnt_last);
                static float    timelineS();
                static float    timelineMs();
                static uint64_t timelineUs();
                static void     delay(float seconds);
                static void     updateSysTime();

                static uint32_t cpuFreqHz() { return cpu_freq_hz_; }

        private:
                static uint32_t cycleCount();

                static uint32_t cpu_freq_hz_;
                static uint32_t cpu_freq_hz_ms_;
                static uint32_t cpu_freq_hz_us_;
                static uint32_t cyccnt_last_;
                static uint64_t cyccnt_round_;
};

#endif //THREE_FRICTION_DWT_HPP
