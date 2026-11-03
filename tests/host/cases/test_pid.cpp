// Step5 对拍：Pid/FuzzyRule/Feedforward/Ldob/TD（C++）vs controller.c（C）
// 注意：两边共享同一个伪 DWT 时间源，必须"各自独立跑完整序列"再比较，
//       交替调用会让后者的 Dt 翻倍。
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <vector>

#include "cotr.hpp"
#include "dwt.hpp"
#include "ref_wrap.h"

extern "C" {
#include "hal_dwt.h"
}

static int g_fail = 0;
#define CHECK_NEAR(a, b, eps, name, i)                                                       \
    do {                                                                                     \
        if (std::fabs((double)(a) - (double)(b)) > (eps)) {                                  \
            printf("FAIL %s[%d]: ref=%.9f new=%.9f\n", name, (i), (double)(b), (double)(a)); \
            g_fail++;                                                                        \
        }                                                                                    \
    } while (0)

static constexpr int kSteps = 600;
static float ref_sig(int i) { return (i < 200) ? 1.0f : ((i < 400) ? -0.6f : 0.25f); }
static float measure_of(int i) { return 0.9f * std::sin(0.02f * (float)i); }

struct PidRec {
    float out, iout, dout, iterm, pout;
};

static std::vector<PidRec> run_ref_pid(uint8_t ImproveMask, bool fuzzy, int n) {
    std::vector<PidRec> v;
    v.reserve(n);

    dwtHostReset(168000U);
    DWT_Init(168U);

    ref_pid_t* rp =
        ref_pid_new(10.0f, 5.0f, 0.0f, 1.2f, 0.8f, 0.02f, 0.5f, 0.5f, 0.01f, 0.01f, 4, ImproveMask);
    ref_fuzzy_t* rf = nullptr;
    if (fuzzy) {
        rf = ref_fuzzy_new(1.0f, 1.0f, 1.0f, 0.5f, 0.5f);
        ref_pid_attach_fuzzy(rp, rf);
    }
    for (int i = 0; i < n; i++) {
        const float m = measure_of(i), r = ref_sig(i);
        if (rf) {
            ref_fuzzy_update(rf, m, r);
        }
        PidRec rec{};
        rec.out = ref_pid_calc(rp, m, r);
        rec.iout = ref_pid_iout(rp);
        rec.dout = ref_pid_dout(rp);
        rec.iterm = ref_pid_iterm(rp);
        rec.pout = ref_pid_pout(rp);
        v.push_back(rec);
    }
    ref_pid_free(rp);
    if (rf) {
        ref_fuzzy_free(rf);
    }
    return v;
}

static std::vector<PidRec> run_new_pid(uint8_t ImproveMask, bool fuzzy, int n) {
    std::vector<PidRec> v;
    v.reserve(n);

    dwtHostReset(168000U);
    Dwt::Init(168U);

    Pid p;
    p.Init(10.0f, 5.0f, 0.0f, 1.2f, 0.8f, 0.02f, 0.5f, 0.5f, 0.01f, 0.01f, 4, ImproveMask);
    if (fuzzy) {
        p.EnableFuzzy(nullptr, nullptr, nullptr, 1.0f, 1.0f, 1.0f, 0.5f, 0.5f);
    }

    for (int i = 0; i < n; i++) {
        const float m = measure_of(i), r = ref_sig(i);
        if (fuzzy) {
            p.UpdateFuzzy(m, r);
        }
        PidRec rec{};
        rec.out = p.Calculate(m, r);
        rec.iout = p.IOut();
        rec.dout = p.DOut();
        rec.iterm = p.ITerm();
        rec.pout = p.POut();
        v.push_back(rec);
    }
    return v;
}

int main() {
    // ── PID：8 个 ImproveMask 位逐个 + 全开 ──
    for (uint8_t ImproveMask :
         {0x00u, 0x01u, 0x02u, 0x04u, 0x08u, 0x10u, 0x20u, 0x40u, 0x80u, 0xFFu}) {
        const auto r = run_ref_pid(ImproveMask, false, kSteps);
        const auto n = run_new_pid(ImproveMask, false, kSteps);
        for (int i = 0; i < kSteps; i++) {
            CHECK_NEAR(n[i].out, r[i].out, 0.0f, "Output", i);
            CHECK_NEAR(n[i].iout, r[i].iout, 0.0f, "iout", i);
            CHECK_NEAR(n[i].dout, r[i].dout, 0.0f, "dout", i);
            CHECK_NEAR(n[i].iterm, r[i].iterm, 0.0f, "iterm", i);
            CHECK_NEAR(n[i].pout, r[i].pout, 0.0f, "pout", i);
            if (g_fail > 4) {
                break;
            }
        }
        if (g_fail > 4) {
            printf("ImproveMask=0x%02X 比对中断\n", ImproveMask);
            break;
        }
    }
    printf("PID ImproveMask 位遍历完成（%d 组）\n", 10);

    // ── 模糊 PID ──
    {
        const auto r = run_ref_pid(0x00u, true, kSteps);
        const auto n = run_new_pid(0x00u, true, kSteps);
        for (int i = 0; i < kSteps; i++) {
            CHECK_NEAR(n[i].out, r[i].out, 0.0f, "fuzzy Output", i);
            if (g_fail > 4) {
                break;
            }
        }
        printf("模糊 PID 对拍完成\n");
    }

    // ── Feedforward ──
    {
        const float c[3] = {1.5f, 0.4f, 0.02f};
        std::vector<float> rv(kSteps), nv(kSteps);

        dwtHostReset(168000U);
        DWT_Init(168U);
        ref_ff_t* rf = ref_ff_new(20.0f, c, 0.01f, 4, 4);
        for (int i = 0; i < kSteps; i++) {
            rv[i] = ref_ff_calc(rf, std::sin(0.01f * (float)i) * 2.0f);
        }
        ref_ff_free(rf);

        dwtHostReset(168000U);
        Dwt::Init(168U);
        Feedforward nf;
        nf.Init(20.0f, c, 0.01f, 4, 4);
        for (int i = 0; i < kSteps; i++) {
            nv[i] = nf.Calculate(std::sin(0.01f * (float)i) * 2.0f);
        }

        for (int i = 0; i < kSteps; i++) {
            CHECK_NEAR(nv[i], rv[i], 0.0f, "ff", i);
        }
        printf("Feedforward 对拍完成\n");
    }

    // ── LDOB ──
    {
        float c[3] = {2.0f, 0.3f, 0.01f};
        std::vector<float> rv(kSteps), nv(kSteps);

        dwtHostReset(168000U);
        DWT_Init(168U);
        ref_ldob_t* rl = ref_ldob_new(5.0f, 0.02f, c, 0.01f, 4, 4);
        for (int i = 0; i < kSteps; i++) {
            rv[i] = ref_ldob_calc(rl, std::sin(0.01f * (float)i), 0.1f * (float)(i % 10));
        }
        ref_ldob_free(rl);

        dwtHostReset(168000U);
        Dwt::Init(168U);
        Ldob nl;
        nl.Init(5.0f, 0.02f, c, 0.01f, 4, 4);
        for (int i = 0; i < kSteps; i++) {
            nv[i] = nl.Calculate(std::sin(0.01f * (float)i), 0.1f * (float)(i % 10));
        }

        for (int i = 0; i < kSteps; i++) {
            CHECK_NEAR(nv[i], rv[i], 0.0f, "ldob", i);
        }
        printf("LDOB 对拍完成\n");
    }

    // ── TD ──
    {
        std::vector<float> rv(kSteps), nv(kSteps);

        dwtHostReset(168000U);
        DWT_Init(168U);
        ref_td_t* rt = ref_td_new(50.0f, 0.01f);
        for (int i = 0; i < kSteps; i++) {
            rv[i] = ref_td_calc(rt, ref_sig(i));
        }
        ref_td_free(rt);

        dwtHostReset(168000U);
        Dwt::Init(168U);
        TrackingDifferentiator nt;
        nt.Init(50.0f, 0.01f);
        for (int i = 0; i < kSteps; i++) {
            nv[i] = nt.Calculate(ref_sig(i));
        }

        for (int i = 0; i < kSteps; i++) {
            CHECK_NEAR(nv[i], rv[i], 0.0f, "td", i);
        }
        printf("TD 对拍完成\n");
    }

    printf(g_fail == 0 ? "test_pid: PASS\n" : "test_pid: %d FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
