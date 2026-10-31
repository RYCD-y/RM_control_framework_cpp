//
// DWT 周期计数器实现（仅固件实现；host 对拍用 tests/host/stubs/dwt_host.cpp）
//
#include "dwt.hpp"

#include "stm32f4xx_hal.h"    // 只用官方入口头

uint32_t Dwt::cpu_freq_hz_    = 0U;
uint32_t Dwt::cpu_freq_hz_ms_ = 0U;
uint32_t Dwt::cpu_freq_hz_us_ = 0U;
uint32_t Dwt::cyccnt_last_    = 0U;
uint64_t Dwt::cyccnt_round_   = 0U;

uint32_t Dwt::cycleCount()
{
        return DWT->CYCCNT;
}

void Dwt::init(uint32_t cpu_freq_mhz)
{
        // 使能 DWT 外设并清零 CYCCNT
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        cpu_freq_hz_    = cpu_freq_mhz * 1000000U;
        cpu_freq_hz_ms_ = cpu_freq_hz_ / 1000U;
        cpu_freq_hz_us_ = cpu_freq_hz_ / 1000000U;
        cyccnt_round_   = 0U;
        cyccnt_last_    = 0U;
}

float Dwt::deltaT(uint32_t &cnt_last)
{
        if (cpu_freq_hz_ == 0U) { return 0.0f; }

        const uint32_t cnt_now = DWT->CYCCNT;
        const float    dt      = ((float) (cnt_now - cnt_last)) / ((float) cpu_freq_hz_);
        cnt_last               = cnt_now;

        updateSysTime();

        return dt;
}

double Dwt::deltaT64(uint32_t &cnt_last)
{
        if (cpu_freq_hz_ == 0U) { return 0.0; }

        const uint32_t cnt_now = DWT->CYCCNT;
        const double   dt      = ((uint32_t) (cnt_now - cnt_last)) / ((double) cpu_freq_hz_);
        cnt_last               = cnt_now;

        updateSysTime();

        return dt;
}

void Dwt::updateSysTime()
{
        const uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < cyccnt_last_) { cyccnt_round_++; }
        cyccnt_last_ = cnt_now;
}

float Dwt::timelineS()
{
        const uint64_t cyccnt64 = cyccnt_round_ * (uint64_t) UINT32_MAX + (uint64_t) DWT->CYCCNT;
        return (float) cyccnt64 / (float) cpu_freq_hz_;
}

float Dwt::timelineMs()
{
        const uint64_t cyccnt64 = cyccnt_round_ * (uint64_t) UINT32_MAX + (uint64_t) DWT->CYCCNT;
        return (float) (cyccnt64 / (uint64_t) cpu_freq_hz_ms_);
}

uint64_t Dwt::timelineUs()
{
        const uint64_t cyccnt64 = cyccnt_round_ * (uint64_t) UINT32_MAX + (uint64_t) DWT->CYCCNT;
        return cyccnt64 / (uint64_t) cpu_freq_hz_us_;
}

void Dwt::delay(float seconds)
{
        const uint32_t tickstart = DWT->CYCCNT;
        const uint32_t wait      = (uint32_t) (seconds * (float) cpu_freq_hz_);

        while ((DWT->CYCCNT - tickstart) < wait) {}
}
