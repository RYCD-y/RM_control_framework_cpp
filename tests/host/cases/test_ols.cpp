// Step3 对拍：OrdinaryLeastSquares（C++）vs OLS_*（C）
#include <cstdio>
#include <cmath>
#include <initializer_list>

#include "ref_wrap.h"
#include "ols.hpp"

static int g_fail = 0;
#define CHECK_NEAR(a, b, eps, name, i)                                                   \
        do {                                                                             \
                if (std::fabs((double) (a) - (double) (b)) > (eps)) {                    \
                        printf("FAIL %s[%d]: ref=%.9f new=%.9f\n", name, (i),            \
                               (double) (b), (double) (a));                              \
                        g_fail++;                                                        \
                }                                                                        \
        } while (0)

static float input(int i) { return std::sin(0.05f * (float) i) * 100.0f + 0.3f * (float) (i % 7); }

int main()
{
        for (uint16_t order : { 3u, 4u, 8u })
        {
                ref_ols_t           *ref = ref_ols_new(order);
                OrdinaryLeastSquares nw;
                nw.init(order);

                for (int i = 0; i < 300; i++)
                {
                        const float dt = 0.001f;
                        const float y  = input(i);

                        const float rd = ref_ols_derivative(ref, dt, y);
                        const float nd = nw.derivative(dt, y);
                        CHECK_NEAR(nd, rd, 0.0f, "derivative", i);
                        CHECK_NEAR(nw.slope(), ref_ols_k(ref), 0.0f, "k", i);

                        const float rs = ref_ols_smooth(ref, dt, y);
                        const float ns = nw.smooth(dt, y);
                        CHECK_NEAR(ns, rs, 0.0f, "smooth", i);
                        CHECK_NEAR(nw.intercept(), ref_ols_b(ref), 0.0f, "b", i);
                }

                printf("order=%u: k=%f b=%f std=%f\n", (unsigned) order,
                       (double) nw.slope(), (double) nw.intercept(), (double) nw.stdDev());
                ref_ols_free(ref);
        }

        printf(g_fail == 0 ? "test_ols: PASS\n" : "test_ols: %d FAILED\n", g_fail);
        return g_fail == 0 ? 0 : 1;
}
