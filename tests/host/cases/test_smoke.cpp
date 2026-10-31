// Step1 基准冒烟：确认 v2 的 C 实现能在 host 上跑起来
#include <cstdio>
#include <cmath>

#include "ref_wrap.h"

static int g_fail = 0;

#define CHECK(cond, fmt, ...)                                            \
        do {                                                             \
                if (!(cond)) { printf("FAIL: " fmt "\n", ##__VA_ARGS__); g_fail++; } \
        } while (0)

int main()
{
        // ── PID 能算出非零输出 ──
        ref_pid_t *pid = ref_pid_new(10.0f, 5.0f, 0.0f, 1.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
        const float out = ref_pid_calc(pid, 0.0f, 1.0f);
        printf("PID  out=%f  dt=%f\n", (double) out, (double) ref_pid_dt(pid));
        CHECK(std::fabs(out - 1.0f) < 1e-3f, "PID 输出异常: %f", (double) out);

        // ── OLS 能拟合出斜率 ──
        ref_ols_t *ols = ref_ols_new(4);
        float d = 0.0f;
        for (int i = 0; i < 100; i++) { d = ref_ols_derivative(ols, 0.001f, (float) i * 0.001f); }
        printf("OLS  derivative=%f  k=%f\n", (double) d, (double) ref_ols_k(ols));
        CHECK(std::fabs(d - 1.0f) < 0.05f, "OLS 导数异常: %f", (double) d);

        // ── CRC 标准向量 ──
        unsigned char buf[16] = { 0x01, 0x02, 0x03, 0x04 };
        const unsigned char c8  = ref_crc8(buf, 4, 0xFF);
        const unsigned short c16 = ref_crc16(buf, 4, 0xFFFF);
        printf("CRC8=0x%02X CRC16=0x%04X\n", c8, c16);
        CHECK(c8 != 0u, "CRC8 为 0，可疑");

        printf(g_fail == 0 ? "test_smoke: PASS\n" : "test_smoke: %d FAILED\n", g_fail);
        return g_fail == 0 ? 0 : 1;
}
