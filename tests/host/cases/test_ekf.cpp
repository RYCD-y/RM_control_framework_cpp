// Step7 对拍：QuaternionEkf（C++）vs QuaternionEKF（C）
#include <cmath>
#include <cstdio>
#include <vector>

#include "quaternion_ekf.hpp"
#include "ref_wrap.h"

static int g_fail = 0;
#define CHECK_NEAR(a, b, eps, name, i)                                                       \
    do {                                                                                     \
        const double d = std::fabs((double)(a) - (double)(b));                               \
        const double s = std::fabs((double)(b)) + 1e-6;                                      \
        if (d > (eps) * s) {                                                                 \
            printf("FAIL %s[%d]: ref=%.9f new=%.9f\n", name, (i), (double)(b), (double)(a)); \
            g_fail++;                                                                        \
        }                                                                                    \
    } while (0)

static constexpr int kSteps = 1000;
static constexpr float kDt = 0.001f;

// 模拟静止 + 缓慢旋转的 IMU
static void imu(int i, float& gx, float& gy, float& gz, float& ax, float& ay, float& az) {
    const float t = (float)i * 0.001f;
    gx = 0.02f * std::sin(1.0f * t);
    gy = 0.01f * std::cos(0.7f * t);
    gz = 0.05f;
    ax = 0.03f * std::sin(2.0f * t);
    ay = 0.02f * std::cos(1.5f * t);
    az = 9.80f;
}

int main() {
    std::vector<float> rq(kSteps * 4), nq(kSteps * 4);
    std::vector<float> rb(kSteps * 3), nb(kSteps * 3);
    std::vector<float> rr(kSteps), nr(kSteps), rp(kSteps), np(kSteps), ry(kSteps), ny(kSteps);

    ref_ekf_t* rk = ref_ekf_new(10.0f, 0.001f, 1000000.0f * 10.0f, 0.9996f, 0.0f);
    for (int i = 0; i < kSteps; i++) {
        float gx, gy, gz, ax, ay, az;
        imu(i, gx, gy, gz, ax, ay, az);
        ref_ekf_update(rk, gx, gy, gz, ax, ay, az, kDt);
        const float* q = ref_ekf_quat(rk);
        const float* b = ref_ekf_bias(rk);
        for (int j = 0; j < 4; j++) {
            rq[i * 4 + j] = q[j];
        }
        for (int j = 0; j < 3; j++) {
            rb[i * 3 + j] = b[j];
        }
        rr[i] = ref_ekf_roll(rk);
        rp[i] = ref_ekf_pitch(rk);
        ry[i] = ref_ekf_yaw(rk);
    }

    QuaternionEkf nk;
    nk.Init(10.0f, 0.001f, 1000000.0f * 10.0f, 0.9996f, 0.0f);
    for (int i = 0; i < kSteps; i++) {
        float gx, gy, gz, ax, ay, az;
        imu(i, gx, gy, gz, ax, ay, az);
        nk.Update(gx, gy, gz, ax, ay, az, kDt);
        for (int j = 0; j < 4; j++) {
            nq[i * 4 + j] = nk.Quaternion()[j];
        }
        for (int j = 0; j < 3; j++) {
            nb[i * 3 + j] = nk.gyroBias()[j];
        }
        nr[i] = nk.RollDeg();
        np[i] = nk.PitchDeg();
        ny[i] = nk.YawDeg();
    }

    for (int i = 0; i < kSteps; i++) {
        for (int j = 0; j < 4; j++) {
            CHECK_NEAR(nq[i * 4 + j], rq[i * 4 + j], 1e-5, "q", i);
        }
        for (int j = 0; j < 3; j++) {
            CHECK_NEAR(nb[i * 3 + j], rb[i * 3 + j], 1e-5, "bias", i);
        }
        CHECK_NEAR(nr[i], rr[i], 1e-5, "RollDeg", i);
        CHECK_NEAR(np[i], rp[i], 1e-5, "PitchDeg", i);
        CHECK_NEAR(ny[i], ry[i], 1e-5, "YawDeg", i);
        if (g_fail > 8) {
            break;
        }
    }

    printf("final: q=[%.5f %.5f %.5f %.5f] bias=[%.5f %.5f]\n", (double)nk.Quaternion()[0],
           (double)nk.Quaternion()[1], (double)nk.Quaternion()[2], (double)nk.Quaternion()[3],
           (double)nk.gyroBias()[0], (double)nk.gyroBias()[1]);
    printf("       RollDeg=%.3f PitchDeg=%.3f YawDeg=%.3f  Converged=%d\n", (double)nk.RollDeg(),
           (double)nk.PitchDeg(), (double)nk.YawDeg(), (int)nk.Converged());

    ref_ekf_free(rk);
    printf(g_fail == 0 ? "test_ekf: PASS\n" : "test_ekf: %d FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
