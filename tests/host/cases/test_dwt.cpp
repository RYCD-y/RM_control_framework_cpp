// Step2 对拍：Dwt（C++）vs hal_dwt（C）
#include <cmath>
#include <cstdio>
#include <cstring>

#include "dwt.hpp"
#include "ref_wrap.h"

extern "C" {
#include "hal_dwt.h"
}

static int g_fail = 0;
#define CHECK_NEAR(a, b, eps, name)                                                 \
    do {                                                                            \
        if (std::fabs((double)(a) - (double)(b)) > (eps)) {                         \
            printf("FAIL %s: ref=%.9f new=%.9f\n", name, (double)(b), (double)(a)); \
            g_fail++;                                                               \
        }                                                                           \
    } while (0)

static constexpr int N = 16;

int main() {
    float ref_dt[N], new_dt[N];

    // 同一个桩时间源：先跑 ref，再 Reset 跑 new
    dwtHostReset(168000U);  // 每次读 CYCCNT 前进 1 ms
    DWT_Init(168U);
    uint32_t last_ref = 0;
    for (int i = 0; i < N; i++) {
        ref_dt[i] = DWT_GetDeltaT(&last_ref);
    }

    dwtHostReset(168000U);
    Dwt::Init(168U);
    uint32_t last_new = 0;
    for (int i = 0; i < N; i++) {
        new_dt[i] = Dwt::DeltaT(last_new);
    }

    for (int i = 0; i < N; i++) {
        CHECK_NEAR(new_dt[i], ref_dt[i], 0.0f, "DeltaT");
    }
    printf("DeltaT[0]=%.9f  DeltaT[15]=%.9f  cpu=%u Hz\n", (double)new_dt[0], (double)new_dt[N - 1],
           Dwt::CpuFreqHz());

    // 时间线
    dwtHostReset(168000U);
    DWT_Init(168U);
    for (int i = 0; i < 10; i++) {
        (void)DWT_GetDeltaT(&last_ref);
    }
    const float ref_ms = DWT_GetTimeline_ms();

    dwtHostReset(168000U);
    Dwt::Init(168U);
    for (int i = 0; i < 10; i++) {
        (void)Dwt::DeltaT(last_new);
    }
    const float new_ms = Dwt::TimelineMs();

    CHECK_NEAR(new_ms, ref_ms, 1e-3f, "TimelineMs");
    printf("timeline: ref=%.3f ms  new=%.3f ms\n", (double)ref_ms, (double)new_ms);

    printf(g_fail == 0 ? "test_dwt: PASS\n" : "test_dwt: %d FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
