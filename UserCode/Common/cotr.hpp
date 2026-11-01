//
// 控制算法：模糊规则 / PID / 前馈 / 线性扰动观测器 / 跟踪微分器
// 对应 control_framework_v2/UserCode/Common/controller.h + controller.c
//
#ifndef THREE_FRICTION_PID_HPP
#define THREE_FRICTION_PID_HPP

#include <stdint.h>

#include "ols.hpp"

class Dwt;

/* ═══════════════════════ 模糊 PID 规则 ═══════════════════════ */
class FuzzyRule
{
        public:
                FuzzyRule() = default;

                /// 传 nullptr 表示使用内置默认规则表
                void init(const float (*rule_kp)[7], const float (*rule_ki)[7], const float (*rule_kd)[7],
                          float kp_ratio, float ki_ratio, float kd_ratio,
                          float e_step, float ec_step);

                void update(float measure, float ref);      // 周期调用，更新隶属度与模糊输出

                float kpFuzzy() const { return kp_fuzzy_; }
                float kiFuzzy() const { return ki_fuzzy_; }
                float kdFuzzy() const { return kd_fuzzy_; }
                float dt() const { return dt_; }

        private:
                const float (*rule_kp_)[7] = nullptr;
                const float (*rule_ki_)[7] = nullptr;
                const float (*rule_kd_)[7] = nullptr;

                float kp_ratio_ = 0.0f, ki_ratio_ = 0.0f, kd_ratio_ = 0.0f;
                float e_step_ = 1.0f, ec_step_ = 1.0f;

                float kp_fuzzy_ = 0.0f, ki_fuzzy_ = 0.0f, kd_fuzzy_ = 0.0f;
                float e_ = 0.0f, ec_ = 0.0f, e_last_ = 0.0f;
                float dt_ = 0.0f;
                uint32_t dwt_cnt_ = 0;
};

/* ═══════════════════════ PID ═══════════════════════ */
class Pid
{
        public:
                using Hook = void (*)(Pid &);

                /// 改进项（可位或）
                enum Improve : uint8_t
                {
                        None                      = 0x00,
                        IntegralLimit             = 0x01,
                        DerivativeOnMeasurement   = 0x02,
                        TrapezoidIntegral         = 0x04,
                        ProportionalOnMeasurement = 0x08,
                        OutputFilter              = 0x10,
                        ChangingIntegrationRate   = 0x20,
                        DerivativeFilter          = 0x40,
                        ErrorHandle               = 0x80,
                };

                enum ErrorType : uint8_t
                {
                        ErrorNone   = 0x00,
                        MotorBlocked = 0x01,
                };

                void init(float max_out, float integral_limit, float deadband,
                          float kp, float ki, float kd,
                          float coef_a, float coef_b,
                          float output_lpf_rc, float derivative_lpf_rc,
                          uint16_t ols_order, uint8_t improve);

                //设置PID参数
                void setGains(float kp, float ki, float kd);
                //设置限幅/死区
                void setLimits(float max_out, float integral_limit, float deadband);
                //注册钩子
                void setHooks(Hook func1, Hook func2);

                /// 启用模糊规则（传 nullptr 用默认规则表）
                void enableFuzzy(const float (*rule_kp)[7], const float (*rule_ki)[7], const float (*rule_kd)[7],
                                 float kp_ratio, float ki_ratio, float kd_ratio,
                                 float e_step, float ec_step);
                void updateFuzzy(float measure, float ref);   // 需要启用模糊时周期调用


                /**
                 *
                 * @param measure 当前值
                 * @param ref 目标值
                 * @return
                 */
                float calculate(float measure, float ref);

                void  reset();

                float    output() const { return output_; }
                float    pOut() const { return p_out_; }
                float    iOut() const { return i_out_; }
                float    dOut() const { return d_out_; }
                float    iTerm() const { return i_term_; }
                float    err() const { return err_; }
                float    measure() const { return measure_; }
                float    dt() const { return dt_; }
                uint8_t  improve() const { return improve_; }
                bool     blocked() const { return error_type_ != ErrorNone; }
                uint64_t errorCount() const { return error_count_; }

        private:
                void trapezoidIntegral();
                void integralLimit();
                void derivativeOnMeasurement();
                void changingIntegrationRate();
                void outputFilter();
                void derivativeFilter();
                void outputLimit();
                void proportionLimit();
                void errorHandle();

                float ref_ = 0.0f, kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
                float measure_ = 0.0f, last_measure_ = 0.0f;
                float err_ = 0.0f, last_err_ = 0.0f, last_i_term_ = 0.0f;
                float p_out_ = 0.0f, i_out_ = 0.0f, d_out_ = 0.0f, i_term_ = 0.0f;
                float output_ = 0.0f, last_output_ = 0.0f, last_d_out_ = 0.0f;
                float max_out_ = 0.0f, integral_limit_ = 0.0f, deadband_ = 0.0f;
                float coef_a_ = 0.0f, coef_b_ = 0.0f;
                float output_lpf_rc_ = 0.0f, derivative_lpf_rc_ = 0.0f;
                float dt_ = 0.0f;
                uint32_t dwt_cnt_ = 0;
                uint8_t  improve_ = None;
                uint8_t  error_type_ = ErrorNone;
                uint64_t error_count_ = 0;

                bool      fuzzy_enabled_ = false;
                FuzzyRule fuzzy_;
                OrdinaryLeastSquares ols_;
                uint16_t  ols_order_ = 0;

                Hook func1_ = nullptr;
                Hook func2_ = nullptr;
};

/* ═══════════════════════ 前馈控制 ═══════════════════════ */
class Feedforward
{
        public:
                void  init(float max_out, const float *c, float lpf_rc,
                           uint16_t ref_dot_ols_order, uint16_t ref_ddot_ols_order);
                float calculate(float ref);

                float output() const { return output_; }
                float dt() const { return dt_; }

        private:
                float    c_[3] = {};
                float    ref_ = 0.0f, last_ref_ = 0.0f;
                float    ref_dot_ = 0.0f, ref_ddot_ = 0.0f, last_ref_dot_ = 0.0f;
                float    lpf_rc_ = 0.0f, max_out_ = 0.0f, output_ = 0.0f, dt_ = 0.0f;
                uint32_t dwt_cnt_ = 0;
                uint16_t ref_dot_ols_order_ = 0, ref_ddot_ols_order_ = 0;
                OrdinaryLeastSquares ref_dot_ols_, ref_ddot_ols_;
};

/* ═══════════════════════ 线性扰动观测器 ═══════════════════════ */
class Ldob
{
        public:
                void  init(float max_disturbance, float deadband, const float *c, float lpf_rc,
                           uint16_t measure_dot_ols_order, uint16_t measure_ddot_ols_order);
                float calculate(float measure, float u);

                float output() const { return output_; }
                float disturbance() const { return disturbance_; }
                float dt() const { return dt_; }

        private:
                float    c_[3] = {};
                float    measure_ = 0.0f, last_measure_ = 0.0f;
                float    measure_dot_ = 0.0f, measure_ddot_ = 0.0f, last_measure_dot_ = 0.0f;
                float    u_ = 0.0f;
                float    disturbance_ = 0.0f, last_disturbance_ = 0.0f, output_ = 0.0f;
                float    max_disturbance_ = 0.0f, deadband_ = 0.0f;
                float    lpf_rc_ = 0.0f, dt_ = 0.0f;
                uint32_t dwt_cnt_ = 0;
                uint16_t measure_dot_ols_order_ = 0, measure_ddot_ols_order_ = 0;
                OrdinaryLeastSquares measure_dot_ols_, measure_ddot_ols_;
};

/* ═══════════════════════ 跟踪微分器 ═══════════════════════ */
class TrackingDifferentiator
{
        public:
                void  init(float r, float h0);
                float calculate(float input);

                float x() const { return x_; }
                float dx() const { return dx_; }
                float dt() const { return dt_; }

        private:
                float    r_ = 0.0f, h0_ = 0.0f;
                float    input_ = 0.0f;
                float    x_ = 0.0f, dx_ = 0.0f, ddx_ = 0.0f;
                float    last_dx_ = 0.0f, last_ddx_ = 0.0f;
                float    dt_ = 0.0f;
                uint32_t dwt_cnt_ = 0;
};

#endif //THREE_FRICTION_PID_HPP
