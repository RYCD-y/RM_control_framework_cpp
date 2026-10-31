//
// 最小二乘（滑动窗口）拟合：求信号斜率 / 平滑值 / 标准偏差
// 对应 control_framework_v2/UserCode/Common/utils.c 的 Ordinary_Least_Squares_t + OLS_*
//
#ifndef THREE_FRICTION_OLS_HPP
#define THREE_FRICTION_OLS_HPP

#include <stdint.h>

class OrdinaryLeastSquares
{
        public:
                OrdinaryLeastSquares() = default;
                ~OrdinaryLeastSquares();

                OrdinaryLeastSquares(const OrdinaryLeastSquares &)            = delete;
                OrdinaryLeastSquares &operator=(const OrdinaryLeastSquares &) = delete;

                void  init(uint16_t order);
                void  update(float deltax, float y);       // 只更新拟合（k/b/std）
                float derivative(float deltax, float y);   // 更新并返回斜率 k
                float smooth(float deltax, float y);       // 更新并返回平滑值 k*x[N-1]+b

                float    slope() const { return k_; }
                float    intercept() const { return b_; }
                float    stdDev() const { return std_dev_; }
                uint16_t order() const { return order_; }
                uint32_t count() const { return count_; }

        private:
                void advance(float deltax, float y);   // 窗口前移 + 压入新样本
                void accumulate();                     // 累加 t[0..3]
                void computeSlope();
                void computeIntercept();
                void computeStdDev();

                uint16_t order_   = 0;
                uint32_t count_   = 0;
                float   *x_       = nullptr;
                float   *y_       = nullptr;
                float    k_       = 0.0f;
                float    b_       = 0.0f;
                float    std_dev_ = 0.0f;
                float    t_[4]    = {};
};

#endif //THREE_FRICTION_OLS_HPP
