// host 替身：伪 CYCCNT，每次被读就前进固定步长，便于 ref/new 两边跑同一时间序列
#include <cmath>
#include <cstdio>

#include "hal_dwt.h"

DWT_Time_t SysTime = {};

namespace {
uint32_t g_cycles = 0;
uint32_t g_step = 168000U;  // 默认 1 ms @168 MHz
uint32_t g_cpu_freq = 168000000U;
}  // namespace

void dwtHostReset(uint32_t cycle_step) {
    g_cycles = 0;
    g_step = cycle_step;
}

uint32_t dwtHostCycles(void) { return g_cycles; }
uint32_t dwtHostCpuFreqHz(void) { return g_cpu_freq; }

extern "C" {

void DWT_Init(uint32_t CPU_Freq_mHz) {
    g_cpu_freq = CPU_Freq_mHz * 1000000U;
    g_cycles = 0;
}

float DWT_GetDeltaT(uint32_t* cnt_last) {
    if (cnt_last == nullptr) {
        return 0.0f;
    }
    g_cycles += g_step;
    const uint32_t now = g_cycles;
    const uint32_t diff = now - *cnt_last;
    *cnt_last = now;
    return (float)diff / (float)g_cpu_freq;
}

double DWT_GetDeltaT64(uint32_t* cnt_last) { return (double)DWT_GetDeltaT(cnt_last); }

float DWT_GetTimeline_s(void) { return (float)g_cycles / (float)g_cpu_freq; }
float DWT_GetTimeline_ms(void) { return DWT_GetTimeline_s() * 1000.0f; }
uint64_t DWT_GetTimeline_us(void) {
    return (uint64_t)((double)g_cycles * 1e6 / (double)g_cpu_freq);
}
void DWT_Delay(float Delay) { g_cycles += (uint32_t)(Delay * (float)g_cpu_freq); }
void DWT_SysTimeUpdate(void) {}

}  // extern "C"
