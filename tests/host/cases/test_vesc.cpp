// VESC 协议纯逻辑测试：帧字节、缩放、ERPM/度每秒、扭矩/电流
#include <cmath>
#include <cstdio>
#include <cstdint>

#include "vesc_protocol.hpp"

static int g_fail = 0;

static void CheckBytes(const char* name, const uint8_t* got, const uint8_t* want, int n) {
    for (int i = 0; i < n; ++i) {
        if (got[i] != want[i]) {
            printf("FAIL %s byte[%d]: got 0x%02X want 0x%02X\n", name, i, got[i], want[i]);
            g_fail++;
            return;
        }
    }
}

static void CheckNear(const char* name, float got, float want, float eps) {
    if (std::fabs(got - want) > eps) {
        printf("FAIL %s: got %.6f want %.6f\n", name, (double)got, (double)want);
        g_fail++;
    }
}

int main() {
    uint8_t buf[4];

    // 电流 1.0 A → 1e3 = 1000 = 0x000003E8
    vesc::PutBeI32(buf, vesc::CurrentToRaw(1.0f, 20.0f));
    const uint8_t want_current[4] = {0x00, 0x00, 0x03, 0xE8};
    CheckBytes("SET_CURRENT(1A)", buf, want_current, 4);

    // 占空比 0.5 → 50000 = 0x0000C350
    vesc::PutBeI32(buf, vesc::DutyToRaw(0.5f));
    const uint8_t want_duty[4] = {0x00, 0x00, 0xC3, 0x50};
    CheckBytes("SET_DUTY(0.5)", buf, want_duty, 4);

    // 转子 90 度 → 9e7 = 0x055D4A80
    vesc::PutBeI32(buf, vesc::RotorDegToRaw(90.0f));
    const uint8_t want_pos[4] = {0x05, 0x5D, 0x4A, 0x80};
    CheckBytes("SET_POS(90deg)", buf, want_pos, 4);

    // 负值按二进制补码大端
    vesc::PutBeI32(buf, -1000);
    const uint8_t want_neg[4] = {0xFF, 0xFF, 0xFC, 0x18};
    CheckBytes("SET_CURRENT(-1A)", buf, want_neg, 4);

    // 电流限幅：300 A 在 20 A 上限下被钳位
    CheckNear("CurrentToRaw clamp", (float)vesc::CurrentToRaw(300.0f, 20.0f), 20000.0f, 0.5f);

    // ERPM ↔ 度每秒（M3508：7 极对，19.2032 减速比）
    const float gear = 3591.0f / 187.0f;
    const int32_t erpm = vesc::RpmToErpm(360.0f, 7, gear);
    CheckNear("RpmToErpm(360)", (float)erpm, 360.0f * gear * 7.0f, 1.5f);
    CheckNear("ErpmToRpm round-trip", vesc::ErpmToRpm(erpm, 7, gear), 360.0f, 0.5f);

    // ERPM 为 0 时速度为 0
    CheckNear("ErpmToRpm(0)", vesc::ErpmToRpm(0, 7, gear), 0.0f, 1e-6f);

    // 除零保护
    CheckNear("ErpmToRpm pole_pairs=0", vesc::ErpmToRpm(1000, 0, gear), 0.0f, 1e-6f);

    // 扭矩 → 电流（M3508 Kt = 0.3 N·m/A）
    CheckNear("TorqueToCurrent(3Nm)", vesc::TorqueToCurrent(3.0f, 0.3f), 10.0f, 1e-4f);
    CheckNear("TorqueToCurrent kt=0", vesc::TorqueToCurrent(3.0f, 0.0f), 0.0f, 1e-6f);

    // STATUS_1 解包：ERPM=1000，电流原始 123（→12.3 A），duty 原始 500（→0.5）
    const uint8_t status[8] = {0x00, 0x00, 0x03, 0xE8, 0x00, 0x7B, 0x01, 0xF4};
    CheckNear("STATUS erpm", (float)vesc::GetBeI32(&status[0]), 1000.0f, 0.5f);
    CheckNear("STATUS current", (float)vesc::GetBeI16(&status[4]) / 10.0f, 12.3f, 1e-4f);
    CheckNear("STATUS duty", (float)vesc::GetBeI16(&status[6]) / 1000.0f, 0.5f, 1e-4f);

    // 负 ERPM 符号正确
    const uint8_t neg_erpm[4] = {0xFF, 0xFF, 0xFC, 0x18};
    CheckNear("STATUS negative erpm", (float)vesc::GetBeI32(neg_erpm), -1000.0f, 0.5f);

    printf(g_fail == 0 ? "test_vesc: PASS\n" : "test_vesc: %d FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
