//
// 最小二乘拟合实现（逐行对应 v2 的 OLS_*，累加顺序保持一致以便对拍 bit-exact）
//
#include "ols.hpp"

#include <cmath>
#include <cstring>

#if defined(HOST_TEST)
#include <cstdlib>
static inline void *olsAlloc(size_t n) { return malloc(n); }
#else
#include "FreeRTOS.h"
static inline void *olsAlloc(size_t n) { return pvPortMalloc(n); }
#endif

OrdinaryLeastSquares::~OrdinaryLeastSquares()
{
        if (x_ != nullptr) { free(x_); x_ = nullptr; }
        if (y_ != nullptr) { free(y_); y_ = nullptr; }
}

void OrdinaryLeastSquares::init(uint16_t order)
{
        if (x_ != nullptr) { free(x_); x_ = nullptr; }
        if (y_ != nullptr) { free(y_); y_ = nullptr; }

        order_ = order;
        count_ = 0;
        k_     = 0.0f;
        b_     = 0.0f;
        std::memset(t_, 0, sizeof(t_));

        // order = 0/1 时无法拟合，给 1 个元素兜底，避免后续越界（原 C 版这里是 UB）
        const uint16_t n = (order < 2U) ? 1U : order;
        x_ = static_cast<float *>(olsAlloc(sizeof(float) * n));
        y_ = static_cast<float *>(olsAlloc(sizeof(float) * n));
        if (x_ != nullptr) { std::memset(x_, 0, sizeof(float) * n); }
        if (y_ != nullptr) { std::memset(y_, 0, sizeof(float) * n); }
}

void OrdinaryLeastSquares::advance(float deltax, float y)
{
        const float temp = x_[1];
        for (uint16_t i = 0; i + 1U < order_; ++i)
        {
                x_[i] = x_[i + 1U] - temp;
                y_[i] = y_[i + 1U];
        }
        x_[order_ - 1U] = x_[order_ - 2U] + deltax;
        y_[order_ - 1U] = y;

        if (count_ < order_) { count_++; }
}

void OrdinaryLeastSquares::accumulate()
{
        t_[0] = 0.0f;
        t_[1] = 0.0f;
        t_[2] = 0.0f;
        t_[3] = 0.0f;
        for (uint16_t i = static_cast<uint16_t>(order_ - count_); i < order_; ++i)
        {
                t_[0] += x_[i] * x_[i];
                t_[1] += x_[i];
                t_[2] += x_[i] * y_[i];
                t_[3] += y_[i];
        }
}

void OrdinaryLeastSquares::computeSlope()
{
        const float den = t_[0] * static_cast<float>(order_) - t_[1] * t_[1];
        k_ = (t_[2] * static_cast<float>(order_) - t_[1] * t_[3]) / den;
}

void OrdinaryLeastSquares::computeIntercept()
{
        const float den = t_[0] * static_cast<float>(order_) - t_[1] * t_[1];
        b_ = (t_[0] * t_[3] - t_[1] * t_[2]) / den;
}

void OrdinaryLeastSquares::computeStdDev()
{
        std_dev_ = 0.0f;
        for (uint16_t i = static_cast<uint16_t>(order_ - count_); i < order_; ++i)
        {
                std_dev_ += std::fabs(k_ * x_[i] + b_ - y_[i]);
        }
        std_dev_ /= static_cast<float>(order_);
}

void OrdinaryLeastSquares::update(float deltax, float y)
{
        if (x_ == nullptr || y_ == nullptr || order_ < 2U) { return; }
        advance(deltax, y);
        accumulate();
        computeSlope();
        computeIntercept();
        computeStdDev();
}

float OrdinaryLeastSquares::derivative(float deltax, float y)
{
        if (x_ == nullptr || y_ == nullptr || order_ < 2U) { return 0.0f; }
        advance(deltax, y);
        accumulate();
        computeSlope();
        computeStdDev();          // 与原实现一致：此处 b_ 仍是上一轮的值
        return k_;
}

float OrdinaryLeastSquares::smooth(float deltax, float y)
{
        if (x_ == nullptr || y_ == nullptr || order_ < 2U) { return 0.0f; }
        advance(deltax, y);
        accumulate();
        computeSlope();
        computeIntercept();
        computeStdDev();
        return k_ * x_[order_ - 1U] + b_;
}
