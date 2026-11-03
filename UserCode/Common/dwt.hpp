//
// DWT 周期计数器（微秒级时间基准）
// 完整移植 control_framework_v2/UserCode/HAL/hal_dwt.{h,c}
//
// C → C++ 的对应关系：
//   DWT_Init()               → Dwt::Init()
//   DWT_GetDeltaT()          → Dwt::DeltaT()
//   DWT_GetDeltaT64()        → Dwt::DeltaT64()
//   DWT_SysTimeUpdate()      → Dwt::UpdateSysTime()
//   DWT_GetTimeline_s/ms/us  → Dwt::TimelineS() / TimelineMs() / TimelineUs()
//   DWT_Delay()              → Dwt::Delay()
//   DWT_Time_t SysTime       → Dwt::SysTime()
//
#ifndef THREE_FRICTION_DWT_HPP
#define THREE_FRICTION_DWT_HPP

#include <stdint.h>

/// 系统时间快照（对应 v2 的 DWT_Time_t）
struct DwtTime {
    uint32_t s;   // 秒
    uint16_t ms;  // 毫秒
    uint16_t us;  // 微秒
};

class Dwt {
   public:
    static void Init(uint32_t cpu_freq_mhz);

    /// 上一次调用至今的时间（秒）。cnt_last 由调用者保存（初值 0）。
    static float DeltaT(uint32_t& cnt_last);
    static double DeltaT64(uint32_t& cnt_last);

    static void UpdateSysTime();
    static float TimelineS();
    static float TimelineMs();
    static uint64_t TimelineUs();

    static void Delay(float seconds);

    /// 最近一次 UpdateSysTime()/timelineXxx() 得到的系统时间快照
    static const DwtTime& SysTime() { return sys_time_; }

    static uint32_t CpuFreqHz() { return cpu_freq_hz_; }
    static uint64_t CycleCount64() { return cyccnt64_; }
    static uint32_t CycleCount();

   private:
    static void CntUpdate();  // CYCCNT 溢出计数（对应 DWT_CNT_Update）

    static uint32_t cpu_freq_hz_;
    static uint32_t cpu_freq_hz_ms_;
    static uint32_t cpu_freq_hz_us_;
    static uint32_t cyccnt_round_count_;  // 溢出次数
    static uint32_t cyccnt_last_;
    static uint64_t cyccnt64_;  // 累计周期数
    static DwtTime sys_time_;
};

#endif  // THREE_FRICTION_DWT_HPP
