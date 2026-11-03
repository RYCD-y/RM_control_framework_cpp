// Step6 对拍：KalmanFilter（C++）vs KalmanFilter_t（C）
#include <cmath>
#include <cstdio>
#include <vector>

#include "kalman_filter.hpp"
#include "ref_wrap.h"

static int g_fail = 0;
#define CHECK_NEAR(a, b, eps, name, i, j)                                              \
    do {                                                                               \
        const double d = std::fabs((double)(a) - (double)(b));                         \
        const double s = std::fabs((double)(b)) + 1e-6;                                \
        if (d > (eps) * s) {                                                           \
            printf("FAIL %s[%d,%d]: ref=%.9f new=%.9f\n", name, (i), (j), (double)(b), \
                   (double)(a));                                                       \
            g_fail++;                                                                  \
        }                                                                              \
    } while (0)

// 3 状态（位置/速度/加速度），无控制量，1 个量测
static constexpr int NX = 3, NU = 0, NZ = 1;
static constexpr int kSteps = 200;

int main() {
    const float Dt = 0.001f;

    const float F[NX * NX] = {
        1.0f, Dt, 0.5f * Dt * Dt, 0.0f, 1.0f, Dt, 0.0f, 0.0f, 1.0f,
    };
    float H[NZ * NX] = {1.0f, 0.0f, 0.0f};
    const float Q[NX * NX] = {
        0.001f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.001f,
    };
    const float Rdiag[NZ] = {0.1f};
    const float P0[NX * NX] = {
        10.0f, 0.0f, 0.0f, 0.0f, 30.0f, 0.0f, 0.0f, 0.0f, 10.0f,
    };
    const float x0[NX] = {0.0f, 0.0f, 0.0f};
    const float minVar[NX] = {0.0f, 0.0f, 0.0f};

    ref_kf_t* rk = ref_kf_new(NX, NU, NZ);
    KalmanFilter nk;
    nk.Init(NX, NU, NZ);

    ref_kf_set_F(rk, F);
    ref_kf_set_H(rk, H);
    ref_kf_set_Q(rk, Q);
    ref_kf_set_Rdiag(rk, Rdiag);
    ref_kf_set_x0(rk, x0);
    ref_kf_set_p0(rk, P0);

    nk.SetStateTransition(F);
    nk.SetMeasurementMatrix(H);
    nk.SetProcessNoise(Q);
    nk.SetMeasurementNoiseDiag(Rdiag);
    nk.SetInitialState(x0);
    nk.SetInitialCovariance(P0);
    nk.SetStateMinVariance(minVar);

    for (int i = 0; i < kSteps; i++) {
        // 真值：加速运动 + 少量噪声；量测只有位置
        const float t = (float)i * Dt;
        const float truth = 0.5f * 2.0f * t * t;
        const float z = truth + 0.05f * std::sin(0.3f * (float)i);

        ref_kf_set_measurement(rk, 0, z);
        nk.SetMeasurement(0, z);

        const float* rs = ref_kf_update(rk);
        const float* ns = nk.Update();

        for (int j = 0; j < NX; j++) {
            CHECK_NEAR(ns[j], rs[j], 1e-5, "xhat", i, j);
        }

        const float* rp = ref_kf_cov(rk);
        const float* np = nk.Covariance();
        for (int j = 0; j < NX * NX; j++) {
            CHECK_NEAR(np[j], rp[j], 1e-5, "P", i, j);
        }

        if (g_fail > 8) {
            break;
        }
    }

    printf("final xhat = [%.6f %.6f %.6f]  (ref=[%.6f %.6f %.6f])\n", (double)nk.State()[0],
           (double)nk.State()[1], (double)nk.State()[2], (double)ref_kf_state(rk)[0],
           (double)ref_kf_state(rk)[1], (double)ref_kf_state(rk)[2]);

    ref_kf_free(rk);
    printf(g_fail == 0 ? "test_kf: PASS\n" : "test_kf: %d FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
