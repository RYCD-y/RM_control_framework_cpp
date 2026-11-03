//
// 最小二乘（滑动窗口）拟合：求信号斜率 / 平滑值 / 标准偏差
// 对应 control_framework_v2/UserCode/Common/utils.c 的 Ordinary_Least_Squares_t + OLS_*
//
#ifndef THREE_FRICTION_OLS_HPP
#define THREE_FRICTION_OLS_HPP

#include <stdint.h>

class OrdinaryLeastSquares {
   public:
    OrdinaryLeastSquares() = default;
    ~OrdinaryLeastSquares();

    OrdinaryLeastSquares(const OrdinaryLeastSquares&) = delete;
    OrdinaryLeastSquares& operator=(const OrdinaryLeastSquares&) = delete;

    void Init(uint16_t Order);
    void Update(float deltax, float y);       // 只更新拟合（k/b/std）
    float Derivative(float deltax, float y);  // 更新并返回斜率 k
    float Smooth(float deltax, float y);      // 更新并返回平滑值 k*X[N-1]+b

    float Slope() const { return k_; }
    float Intercept() const { return b_; }
    float StdDev() const { return std_dev_; }
    uint16_t Order() const { return order_; }
    uint32_t Count() const { return count_; }

   private:
    void Advance(float deltax, float y);  // 窗口前移 + 压入新样本
    void Accumulate();                    // 累加 t[0..3]
    void ComputeSlope();
    void ComputeIntercept();
    void ComputeStdDev();

    uint16_t order_ = 0;
    uint32_t count_ = 0;
    float* x_ = nullptr;
    float* y_ = nullptr;
    float k_ = 0.0f;
    float b_ = 0.0f;
    float std_dev_ = 0.0f;
    float t_[4] = {};
};

#endif  // THREE_FRICTION_OLS_HPP
