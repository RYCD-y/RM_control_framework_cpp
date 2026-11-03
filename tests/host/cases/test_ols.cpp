// Step3 对拍：OrdinaryLeastSquares（C++）vs OLS_*（C）
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "ols.hpp"
#include "ref_wrap.h"

static int g_fail = 0;
#define CHECK_NEAR(a, b, eps, name, i)                                                       \
    do {                                                                                     \
        if (std::fabs((double)(a) - (double)(b)) > (eps)) {                                  \
            printf("FAIL %s[%d]: ref=%.9f new=%.9f\n", name, (i), (double)(b), (double)(a)); \
            g_fail++;                                                                        \
        }                                                                                    \
    } while (0)

static float input(int i) { return std::sin(0.05f * (float)i) * 100.0f + 0.3f * (float)(i % 7); }

int main() {
    for (uint16_t Order : {3u, 4u, 8u}) {
        ref_ols_t* ref = ref_ols_new(Order);
        OrdinaryLeastSquares nw;
        nw.Init(Order);

        for (int i = 0; i < 300; i++) {
            const float Dt = 0.001f;
            const float y = input(i);

            const float rd = ref_ols_derivative(ref, Dt, y);
            const float nd = nw.Derivative(Dt, y);
            CHECK_NEAR(nd, rd, 0.0f, "Derivative", i);
            CHECK_NEAR(nw.Slope(), ref_ols_k(ref), 0.0f, "k", i);

            const float rs = ref_ols_smooth(ref, Dt, y);
            const float ns = nw.Smooth(Dt, y);
            CHECK_NEAR(ns, rs, 0.0f, "Smooth", i);
            CHECK_NEAR(nw.Intercept(), ref_ols_b(ref), 0.0f, "b", i);
        }

        printf("Order=%u: k=%f b=%f std=%f\n", (unsigned)Order, (double)nw.Slope(),
               (double)nw.Intercept(), (double)nw.StdDev());
        ref_ols_free(ref);
    }

    printf(g_fail == 0 ? "test_ols: PASS\n" : "test_ols: %d FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
