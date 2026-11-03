//
// 控制算法实现（逐行对应 v2 的 controller.c，运算顺序保持一致以便对拍 bit-exact）
//
#include "cotr.hpp"

#include <algorithm>
#include <cmath>

#include "dwt.hpp"
#include "utils.hpp"

/* ═══════════════════════ 默认模糊规则表（NB..PB = -3..3） ═══════════════════════ */
namespace {
constexpr float kDefaultRuleKp[7][7] = {
    {3, 3, 2, 2, 1, 0, 0}, {3, 3, 2, 1, 1, 0, 1}, {2, 2, 2, 1, 0, 1, 1}, {2, 2, 1, 0, 1, 2, 2},
    {1, 1, 0, 1, 1, 2, 2}, {1, 0, 1, 2, 2, 2, 3}, {0, 0, 2, 2, 2, 3, 3},
};

constexpr float kDefaultRuleKi[7][7] = {
    {3, 3, 2, 2, 1, 0, 0}, {3, 3, 2, 1, 1, 0, 0}, {3, 2, 2, 1, 0, 1, 1}, {2, 2, 1, 0, 1, 2, 2},
    {1, 1, 0, 1, 1, 2, 3}, {0, 0, 1, 1, 2, 3, 3}, {0, 0, 1, 2, 2, 3, 3},
};

constexpr float kDefaultRuleKd[7][7] = {
    {1, 1, 3, 3, 3, 2, 1}, {1, 1, 3, 2, 2, 1, 0}, {0, 1, 2, 2, 1, 1, 0}, {0, 1, 1, 1, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0}, {3, 1, 1, 1, 1, 1, 3}, {3, 2, 2, 2, 1, 1, 3},
};

inline void copy3(float dst[3], const float* src) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}
}  // namespace

/* ═══════════════════════ FuzzyRule ═══════════════════════ */
void FuzzyRule::Init(const float (*rule_kp)[7], const float (*rule_ki)[7],
                     const float (*rule_kd)[7], float kp_ratio, float ki_ratio, float kd_ratio,
                     float e_step, float ec_step) {
    rule_kp_ = (rule_kp == nullptr) ? kDefaultRuleKp : rule_kp;
    rule_ki_ = (rule_ki == nullptr) ? kDefaultRuleKi : rule_ki;
    rule_kd_ = (rule_kd == nullptr) ? kDefaultRuleKd : rule_kd;

    kp_ratio_ = kp_ratio;
    ki_ratio_ = ki_ratio;
    kd_ratio_ = kd_ratio;

    e_step_ = (e_step < 0.00001f) ? 1.0f : e_step;
    ec_step_ = (ec_step < 0.00001f) ? 1.0f : ec_step;
}

void FuzzyRule::Update(float Measure, float ref) {
    dt_ = Dwt::DeltaT(dwt_cnt_);

    e_ = ref - Measure;
    ec_ = (e_ - e_last_) / dt_;
    e_last_ = e_;

    const float e3 = 3.0f * e_step_;
    const float ec3 = 3.0f * ec_step_;

    const int e_left_index =
        (e_ >= e3)
            ? 6
            : ((e_ <= -e3) ? 0
                           : ((e_ >= 0.0f) ? (int)(e_ / e_step_) + 3 : (int)(e_ / e_step_) + 2));
    const int e_right_index =
        (e_ >= e3)
            ? 6
            : ((e_ <= -e3) ? 0
                           : ((e_ >= 0.0f) ? (int)(e_ / e_step_) + 4 : (int)(e_ / e_step_) + 3));
    const int ec_left_index = (ec_ >= ec3)
                                  ? 6
                                  : ((ec_ <= -ec3) ? 0
                                                   : ((ec_ >= 0.0f) ? (int)(ec_ / ec_step_) + 3
                                                                    : (int)(ec_ / ec_step_) + 2));
    const int ec_right_index = (ec_ >= ec3)
                                   ? 6
                                   : ((ec_ <= -ec3) ? 0
                                                    : ((ec_ >= 0.0f) ? (int)(ec_ / ec_step_) + 4
                                                                     : (int)(ec_ / ec_step_) + 3));

    const float e_left_temp =
        (e_ >= e3) ? 0.0f : ((e_ <= -e3) ? 1.0f : ((float)e_right_index - e_ / e_step_ - 3.0f));
    const float e_right_temp =
        (e_ >= e3) ? 1.0f : ((e_ <= -e3) ? 0.0f : (e_ / e_step_ - (float)e_left_index + 3.0f));
    const float ec_left_temp =
        (ec_ >= ec3) ? 0.0f
                     : ((ec_ <= -ec3) ? 1.0f : ((float)ec_right_index - ec_ / ec_step_ - 3.0f));
    const float ec_right_temp =
        (ec_ >= ec3) ? 1.0f
                     : ((ec_ <= -ec3) ? 0.0f : (ec_ / ec_step_ - (float)ec_left_index + 3.0f));

    kp_fuzzy_ = e_left_temp * ec_left_temp * rule_kp_[e_left_index][ec_left_index] +
                e_left_temp * ec_right_temp * rule_kp_[e_right_index][ec_left_index] +
                e_right_temp * ec_left_temp * rule_kp_[e_left_index][ec_right_index] +
                e_right_temp * ec_right_temp * rule_kp_[e_right_index][ec_right_index];

    ki_fuzzy_ = e_left_temp * ec_left_temp * rule_ki_[e_left_index][ec_left_index] +
                e_left_temp * ec_right_temp * rule_ki_[e_right_index][ec_left_index] +
                e_right_temp * ec_left_temp * rule_ki_[e_left_index][ec_right_index] +
                e_right_temp * ec_right_temp * rule_ki_[e_right_index][ec_right_index];

    kd_fuzzy_ = e_left_temp * ec_left_temp * rule_kd_[e_left_index][ec_left_index] +
                e_left_temp * ec_right_temp * rule_kd_[e_right_index][ec_left_index] +
                e_right_temp * ec_left_temp * rule_kd_[e_left_index][ec_right_index] +
                e_right_temp * ec_right_temp * rule_kd_[e_right_index][ec_right_index];
}

/* ═══════════════════════ Pid ═══════════════════════ */
void Pid::Init(float max_out, float integral_limit, float deadband, float kp, float ki, float kd,
               float coef_a, float coef_b, float output_lpf_rc, float derivative_lpf_rc,
               uint16_t ols_order, uint8_t ImproveMask) {
    deadband_ = deadband;
    integral_limit_ = integral_limit;
    max_out_ = max_out;
    ref_ = 0.0f;

    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    i_term_ = 0.0f;

    coef_a_ = coef_a;
    coef_b_ = coef_b;

    output_lpf_rc_ = output_lpf_rc;
    derivative_lpf_rc_ = derivative_lpf_rc;

    ols_order_ = ols_order;
    ols_.Init(ols_order);

    dwt_cnt_ = 0;
    improve_ = ImproveMask;

    error_count_ = 0;
    error_type_ = ErrorNone;

    output_ = 0.0f;
}

void Pid::SetGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void Pid::SetLimits(float max_out, float integral_limit, float deadband) {
    max_out_ = max_out;
    integral_limit_ = integral_limit;
    deadband_ = deadband;
}

void Pid::SetHooks(Hook func1, Hook func2) {
    func1_ = func1;
    func2_ = func2;
}

void Pid::EnableFuzzy(const float (*rule_kp)[7], const float (*rule_ki)[7],
                      const float (*rule_kd)[7], float kp_ratio, float ki_ratio, float kd_ratio,
                      float e_step, float ec_step) {
    fuzzy_.Init(rule_kp, rule_ki, rule_kd, kp_ratio, ki_ratio, kd_ratio, e_step, ec_step);
    fuzzy_enabled_ = true;
}

void Pid::UpdateFuzzy(float Measure, float ref) {
    if (fuzzy_enabled_) {
        fuzzy_.Update(Measure, ref);
    }
}

float Pid::Calculate(float Measure, float ref) {
    if ((improve_ & kErrorHandle) != 0U) {
        ApplyErrorHandle();
    }

    dt_ = Dwt::DeltaT(dwt_cnt_);
    measure_ = Measure;
    ref_ = ref;
    err_ = ref_ - measure_;

    if (func1_ != nullptr) {
        func1_(*this);
    }

    if (std::fabs(err_) > deadband_) {
        if (!fuzzy_enabled_) {
            p_out_ = kp_ * err_;
            i_term_ = ki_ * err_ * dt_;
            if (ols_order_ > 2U) {
                d_out_ = kd_ * ols_.Derivative(dt_, err_);
            } else {
                d_out_ = kd_ * (err_ - last_err_) / dt_;
            }
        } else {
            p_out_ = (kp_ + fuzzy_.KpFuzzy()) * err_;
            i_term_ = (ki_ + fuzzy_.KiFuzzy()) * err_ * dt_;
            if (ols_order_ > 2U) {
                d_out_ = (kd_ + fuzzy_.KdFuzzy()) * ols_.Derivative(dt_, err_);
            } else {
                d_out_ = (kd_ + fuzzy_.KdFuzzy()) * (err_ - last_err_) / dt_;
            }
        }

        if (func2_ != nullptr) {
            func2_(*this);
        }

        if ((improve_ & kTrapezoidIntegral) != 0U) {
            ApplyTrapezoidIntegral();
        }
        if ((improve_ & kChangingIntegrationRate) != 0U) {
            ApplyChangingIntegrationRate();
        }
        if ((improve_ & kDerivativeOnMeasurement) != 0U) {
            ApplyDerivativeOnMeasurement();
        }
        if ((improve_ & kDerivativeFilter) != 0U) {
            ApplyDerivativeFilter();
        }
        if ((improve_ & kIntegralLimit) != 0U) {
            ApplyIntegralLimit();
        }

        i_out_ += i_term_;

        output_ = p_out_ + i_out_ + d_out_;

        if ((improve_ & kOutputFilter) != 0U) {
            ApplyOutputFilter();
        }

        ApplyOutputLimit();
        ApplyProportionLimit();
    }

    last_measure_ = measure_;
    last_output_ = output_;
    last_d_out_ = d_out_;
    last_err_ = err_;
    last_i_term_ = i_term_;

    return output_;
}

void Pid::Reset() {
    i_term_ = 0.0f;
    last_i_term_ = 0.0f;
    i_out_ = 0.0f;

    p_out_ = 0.0f;
    d_out_ = 0.0f;
    last_d_out_ = 0.0f;
    output_ = 0.0f;
    last_output_ = 0.0f;

    err_ = 0.0f;
    last_err_ = 0.0f;
    last_measure_ = 0.0f;

    error_count_ = 0;
    error_type_ = ErrorNone;
}

void Pid::ApplyTrapezoidIntegral() {
    if (!fuzzy_enabled_) {
        i_term_ = ki_ * ((err_ + last_err_) / 2.0f) * dt_;
    } else {
        i_term_ = (ki_ + fuzzy_.KiFuzzy()) * ((err_ + last_err_) / 2.0f) * dt_;
    }
}

void Pid::ApplyChangingIntegrationRate() {
    if (err_ * i_out_ > 0.0f) {
        if (std::fabs(err_) <= coef_b_) {
            return;
        }
        if (std::fabs(err_) <= (coef_a_ + coef_b_)) {
            i_term_ *= (coef_a_ - std::fabs(err_) + coef_b_) / coef_a_;
        } else {
            i_term_ = 0.0f;
        }
    }
}

void Pid::ApplyIntegralLimit() {
    const float temp_i_out = i_out_ + i_term_;
    const float temp_output = p_out_ + i_out_ + d_out_;

    if (std::fabs(temp_output) > max_out_) {
        if (err_ * i_out_ > 0.0f) {
            i_term_ = 0.0f;
        }
    }
    if (temp_i_out > integral_limit_) {
        i_term_ = 0.0f;
        i_out_ = integral_limit_;
    }
    if (temp_i_out < -integral_limit_) {
        i_term_ = 0.0f;
        i_out_ = -integral_limit_;
    }
}

void Pid::ApplyDerivativeOnMeasurement() {
    if (!fuzzy_enabled_) {
        if (ols_order_ > 2U) {
            d_out_ = kd_ * ols_.Derivative(dt_, -measure_);
        } else {
            d_out_ = kd_ * (last_measure_ - measure_) / dt_;
        }
    } else {
        if (ols_order_ > 2U) {
            d_out_ = (kd_ + fuzzy_.KdFuzzy()) * ols_.Derivative(dt_, -measure_);
        } else {
            d_out_ = (kd_ + fuzzy_.KdFuzzy()) * (last_measure_ - measure_) / dt_;
        }
    }
}

void Pid::ApplyDerivativeFilter() {
    d_out_ = d_out_ * dt_ / (derivative_lpf_rc_ + dt_) +
             last_d_out_ * derivative_lpf_rc_ / (derivative_lpf_rc_ + dt_);
}

void Pid::ApplyOutputFilter() {
    output_ = output_ * dt_ / (output_lpf_rc_ + dt_) +
              last_output_ * output_lpf_rc_ / (output_lpf_rc_ + dt_);
}

void Pid::ApplyOutputLimit() {
    if (output_ > max_out_) {
        output_ = max_out_;
    }
    if (output_ < -max_out_) {
        output_ = -max_out_;
    }
}

void Pid::ApplyProportionLimit() {
    if (p_out_ > max_out_) {
        p_out_ = max_out_;
    }
    if (p_out_ < -max_out_) {
        p_out_ = -max_out_;
    }
}

void Pid::ApplyErrorHandle() {
    // ① 输出必须接近饱和：正常调节和加速过程中 PID 不会长时间顶满输出
    if (std::fabs(output_) < max_out_ * kBlockedOutRatio) {
        error_count_ = 0;
        return;
    }

    // ② 实际转速必须"没转起来"：加速时转速会迅速越过这个阈值，
    //    所以目标突变/阶跃只会累计几个周期就被清零，不会误判成堵转
    if (std::fabs(measure_) > kBlockedSpeedMax) {
        error_count_ = 0;
        return;
    }

    error_count_++;

    if (error_count_ > kBlockedCount) {
        error_type_ = MotorBlocked;
    }
}

/* ═══════════════════════ Feedforward ═══════════════════════ */
void Feedforward::Init(float max_out, const float* c, float lpf_rc, uint16_t ref_dot_ols_order,
                       uint16_t ref_ddot_ols_order) {
    max_out_ = max_out;

    if (c != nullptr) {
        copy3(c_, c);
    } else {
        c_[0] = c_[1] = c_[2] = 0.0f;
        max_out_ = 0.0f;
    }

    lpf_rc_ = lpf_rc;

    ref_dot_ols_order_ = ref_dot_ols_order;
    ref_ddot_ols_order_ = ref_ddot_ols_order;
    if (ref_dot_ols_order > 2U) {
        ref_dot_ols_.Init(ref_dot_ols_order);
    }
    if (ref_ddot_ols_order > 2U) {
        ref_ddot_ols_.Init(ref_ddot_ols_order);
    }

    dwt_cnt_ = 0;
    output_ = 0.0f;
}

float Feedforward::Calculate(float ref) {
    dt_ = Dwt::DeltaT(dwt_cnt_);

    ref_ = ref * dt_ / (lpf_rc_ + dt_) + ref_ * lpf_rc_ / (lpf_rc_ + dt_);

    if (ref_dot_ols_order_ > 2U) {
        ref_dot_ = ref_dot_ols_.Derivative(dt_, ref_);
    } else {
        ref_dot_ = (ref_ - last_ref_) / dt_;
    }

    if (ref_ddot_ols_order_ > 2U) {
        ref_ddot_ = ref_ddot_ols_.Derivative(dt_, ref_dot_);
    } else {
        ref_ddot_ = (ref_dot_ - last_ref_dot_) / dt_;
    }

    output_ = c_[0] * ref_ + c_[1] * ref_dot_ + c_[2] * ref_ddot_;
    output_ = std::clamp(output_, -max_out_, max_out_);

    last_ref_ = ref_;
    last_ref_dot_ = ref_dot_;

    return output_;
}

/* ═══════════════════════ Ldob ═══════════════════════ */
void Ldob::Init(float max_disturbance, float deadband, const float* c, float lpf_rc,
                uint16_t measure_dot_ols_order, uint16_t measure_ddot_ols_order) {
    max_disturbance_ = max_disturbance;
    deadband_ = deadband;

    if (c != nullptr) {
        copy3(c_, c);
    } else {
        c_[0] = c_[1] = c_[2] = 0.0f;
        max_disturbance_ = 0.0f;
    }

    lpf_rc_ = lpf_rc;

    measure_dot_ols_order_ = measure_dot_ols_order;
    measure_ddot_ols_order_ = measure_ddot_ols_order;
    if (measure_dot_ols_order > 2U) {
        measure_dot_ols_.Init(measure_dot_ols_order);
    }
    if (measure_ddot_ols_order > 2U) {
        measure_ddot_ols_.Init(measure_ddot_ols_order);
    }

    dwt_cnt_ = 0;
    disturbance_ = 0.0f;
}

float Ldob::Calculate(float Measure, float u) {
    dt_ = Dwt::DeltaT(dwt_cnt_);

    measure_ = Measure;
    u_ = u;

    if (measure_dot_ols_order_ > 2U) {
        measure_dot_ = measure_dot_ols_.Derivative(dt_, measure_);
    } else {
        measure_dot_ = (measure_ - last_measure_) / dt_;
    }

    if (measure_ddot_ols_order_ > 2U) {
        measure_ddot_ = measure_ddot_ols_.Derivative(dt_, measure_dot_);
    } else {
        measure_ddot_ = (measure_dot_ - last_measure_dot_) / dt_;
    }

    disturbance_ = c_[0] * measure_ + c_[1] * measure_dot_ + c_[2] * measure_ddot_ - u_;
    disturbance_ =
        disturbance_ * dt_ / (lpf_rc_ + dt_) + last_disturbance_ * lpf_rc_ / (lpf_rc_ + dt_);

    disturbance_ = std::clamp(disturbance_, -max_disturbance_, max_disturbance_);

    if (std::fabs(disturbance_) > deadband_ * max_disturbance_) {
        output_ = disturbance_;
    } else {
        output_ = 0.0f;
    }

    last_measure_ = measure_;
    last_measure_dot_ = measure_dot_;
    last_disturbance_ = disturbance_;

    return output_;
}

/* ═══════════════════════ TrackingDifferentiator ═══════════════════════ */
void TrackingDifferentiator::Init(float r, float h0) {
    r_ = r;
    h0_ = h0;

    x_ = 0.0f;
    dx_ = 0.0f;
    ddx_ = 0.0f;
    last_dx_ = 0.0f;
    last_ddx_ = 0.0f;
}

float TrackingDifferentiator::Calculate(float input) {
    dt_ = Dwt::DeltaT(dwt_cnt_);

    if (dt_ > 0.5f) {
        return 0.0f;
    }

    input_ = input;

    const float d = r_ * h0_ * h0_;
    const float a0 = dx_ * h0_;
    const float y = x_ - input_ + a0;
    const float a1 = std::sqrt(d * (d + 8.0f * std::fabs(y)));
    const float a2 = a0 + utils::Sign(y) * (a1 - d) / 2.0f;
    const float a = (a0 + y) * (utils::Sign(y + d) - utils::Sign(y - d)) / 2.0f +
                    a2 * (1.0f - (utils::Sign(y + d) - utils::Sign(y - d)) / 2.0f);
    const float fhan =
        -r_ * a / d * (utils::Sign(a + d) - utils::Sign(a - d)) / 2.0f -
        r_ * utils::Sign(a) * (1.0f - (utils::Sign(a + d) - utils::Sign(a - d)) / 2.0f);

    ddx_ = fhan;
    dx_ += (ddx_ + last_ddx_) * dt_ / 2.0f;
    x_ += (dx_ + last_dx_) * dt_ / 2.0f;

    last_ddx_ = ddx_;
    last_dx_ = dx_;

    return x_;
}
