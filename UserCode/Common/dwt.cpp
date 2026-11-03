//
// DWT 周期计数器实现（仅固件实现；host 对拍用 tests/host/stubs/dwt_host.cpp）
// 逐行对应 v2 的 hal_dwt.c
//
#include "dwt.hpp"

#include "stm32f4xx_hal.h"  // 只用官方入口头

uint32_t Dwt::cpu_freq_hz_ = 0U;
uint32_t Dwt::cpu_freq_hz_ms_ = 0U;
uint32_t Dwt::cpu_freq_hz_us_ = 0U;
uint32_t Dwt::cyccnt_round_count_ = 0U;
uint32_t Dwt::cyccnt_last_ = 0U;
uint64_t Dwt::cyccnt64_ = 0U;
DwtTime Dwt::sys_time_ = {0U, 0U, 0U};

uint32_t Dwt::CycleCount() { return DWT->CYCCNT; }

void Dwt::Init(uint32_t cpu_freq_mhz) {
    /* 使能 DWT 外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT 寄存器计数清 0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能 Cortex-M DWT CYCCNT 寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cpu_freq_hz_ = cpu_freq_mhz * 1000000U;
    cpu_freq_hz_ms_ = cpu_freq_hz_ / 1000U;
    cpu_freq_hz_us_ = cpu_freq_hz_ / 1000000U;
    cyccnt_round_count_ = 0U;
    cyccnt_last_ = 0U;
    cyccnt64_ = 0U;
    sys_time_ = {0U, 0U, 0U};
}

void Dwt::CntUpdate() {
    const uint32_t cnt_now = DWT->CYCCNT;

    if (cnt_now < cyccnt_last_) {
        cyccnt_round_count_++;
    }

    cyccnt_last_ = cnt_now;
}

float Dwt::DeltaT(uint32_t& cnt_last) {
    const uint32_t cnt_now = DWT->CYCCNT;
    const float Dt = ((float)(cnt_now - cnt_last)) / ((float)cpu_freq_hz_);
    cnt_last = cnt_now;

    CntUpdate();

    return Dt;
}

double Dwt::DeltaT64(uint32_t& cnt_last) {
    const uint32_t cnt_now = DWT->CYCCNT;
    const double Dt = ((uint32_t)(cnt_now - cnt_last)) / ((double)cpu_freq_hz_);
    cnt_last = cnt_now;

    CntUpdate();

    return Dt;
}

void Dwt::UpdateSysTime() {
    const uint32_t cnt_now = DWT->CYCCNT;

    CntUpdate();

    cyccnt64_ = (uint64_t)cyccnt_round_count_ * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;

    const uint64_t cnt_temp1 = cyccnt64_ / cpu_freq_hz_;
    const uint64_t cnt_temp2 = cyccnt64_ - cnt_temp1 * cpu_freq_hz_;

    sys_time_.s = (uint32_t)cnt_temp1;
    sys_time_.ms = (uint16_t)(cnt_temp2 / cpu_freq_hz_ms_);

    const uint64_t cnt_temp3 = cnt_temp2 - (uint64_t)sys_time_.ms * cpu_freq_hz_ms_;
    sys_time_.us = (uint16_t)(cnt_temp3 / cpu_freq_hz_us_);
}

float Dwt::TimelineS() {
    UpdateSysTime();

    return (float)sys_time_.s + (float)sys_time_.ms * 0.001f + (float)sys_time_.us * 0.000001f;
}

float Dwt::TimelineMs() {
    UpdateSysTime();

    return (float)sys_time_.s * 1000.0f + (float)sys_time_.ms + (float)sys_time_.us * 0.001f;
}

uint64_t Dwt::TimelineUs() {
    UpdateSysTime();

    return (uint64_t)sys_time_.s * 1000000ULL + (uint64_t)sys_time_.ms * 1000ULL +
           (uint64_t)sys_time_.us;
}

void Dwt::Delay(float seconds) {
    const uint32_t tickstart = DWT->CYCCNT;

    while ((float)(DWT->CYCCNT - tickstart) < seconds * (float)cpu_freq_hz_) {
    }
}
