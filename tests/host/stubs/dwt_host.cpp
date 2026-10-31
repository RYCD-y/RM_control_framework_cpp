// host 替身：Dwt 类的非固件实现（替代 UserCode/Common/dwt.cpp）
// 时间源与 hal_dwt 的 host 桩共用，保证与 ref 走同一时间序列。
#include "dwt.hpp"
#include "hal_dwt.h"

uint32_t Dwt::cpu_freq_hz_        = 0U;
uint32_t Dwt::cpu_freq_hz_ms_     = 0U;
uint32_t Dwt::cpu_freq_hz_us_     = 0U;
uint32_t Dwt::cyccnt_round_count_ = 0U;
uint32_t Dwt::cyccnt_last_        = 0U;
uint64_t Dwt::cyccnt64_           = 0U;
DwtTime  Dwt::sys_time_           = { 0U, 0U, 0U };

uint32_t Dwt::cycleCount() { return dwtHostCycles(); }
void Dwt::cntUpdate() {}

void Dwt::init(uint32_t cpu_freq_mhz)
{
        DWT_Init(cpu_freq_mhz);
        cpu_freq_hz_    = cpu_freq_mhz * 1000000U;
        cpu_freq_hz_ms_ = cpu_freq_hz_ / 1000U;
        cpu_freq_hz_us_ = cpu_freq_hz_ / 1000000U;
        cyccnt_round_count_ = 0U;
        cyccnt_last_        = 0U;
        cyccnt64_           = 0U;
        sys_time_           = { 0U, 0U, 0U };
}

float Dwt::deltaT(uint32_t &cnt_last) { return DWT_GetDeltaT(&cnt_last); }
double Dwt::deltaT64(uint32_t &cnt_last) { return DWT_GetDeltaT64(&cnt_last); }
void Dwt::updateSysTime() {}
float Dwt::timelineS() { return DWT_GetTimeline_s(); }
float Dwt::timelineMs() { return DWT_GetTimeline_ms(); }
uint64_t Dwt::timelineUs() { return DWT_GetTimeline_us(); }
void Dwt::delay(float seconds) { DWT_Delay(seconds); }
