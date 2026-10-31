// host 替身：可控步进的 DWT（ref 与 new 共用同一时间源）
#ifndef HAL_DWT_HOST_STUB_H
#define HAL_DWT_HOST_STUB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
        uint32_t s;
        uint16_t ms;
        uint16_t us;
} DWT_Time_t;

void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);
extern DWT_Time_t SysTime;

/* ---- host 专用控制接口 ---- */
void     dwtHostReset(uint32_t cycle_step);   // 每次读 CYCCNT 前进 cycle_step
uint32_t dwtHostCycles(void);                 // 当前伪 CYCCNT
uint32_t dwtHostCpuFreqHz(void);

#ifdef __cplusplus
}
#endif

#endif
