#include "ref_wrap.h"

#include <stdlib.h>
#include <string.h>

#include "controller.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "utils.h"
#include "crc8_crc16.h"

struct ref_pid  { PID_t p; };
struct ref_ols  { Ordinary_Least_Squares_t o; };
struct ref_fuzzy { FuzzyRule_t z; };
struct ref_ff   { Feedforward_t f; };
struct ref_ldob { LDOB_t l; };
struct ref_td   { TD_t t; };
struct ref_kf   { KalmanFilter_t k; };
struct ref_ekf  { int dummy; };

/* ── PID ── */
ref_pid_t *ref_pid_new(float max_out, float integral_limit, float deadband,
                       float kp, float ki, float kd, float a, float b,
                       float output_lpf_rc, float derivative_lpf_rc,
                       unsigned short ols_order, unsigned char improve)
{
        ref_pid_t *r = (ref_pid_t *) calloc(1, sizeof(ref_pid_t));
        if (!r) return NULL;
        PID_Init(&r->p, max_out, integral_limit, deadband, kp, ki, kd, a, b,
                 output_lpf_rc, derivative_lpf_rc, ols_order, improve);
        return r;
}
void  ref_pid_free(ref_pid_t *r) { free(r); }
float ref_pid_calc(ref_pid_t *r, float measure, float ref) { return PID_Calculate(&r->p, measure, ref); }
void  ref_pid_reset(ref_pid_t *r) { PID_Reset(&r->p); }
void  ref_pid_set_gains(ref_pid_t *r, float kp, float ki, float kd) { r->p.Kp = kp; r->p.Ki = ki; r->p.Kd = kd; }
float ref_pid_output(ref_pid_t *r) { return r->p.Output; }
float ref_pid_pout(ref_pid_t *r) { return r->p.Pout; }
float ref_pid_iout(ref_pid_t *r) { return r->p.Iout; }
float ref_pid_dout(ref_pid_t *r) { return r->p.Dout; }
float ref_pid_iterm(ref_pid_t *r) { return r->p.ITerm; }
float ref_pid_err(ref_pid_t *r) { return r->p.Err; }
float ref_pid_dt(ref_pid_t *r) { return r->p.dt; }
unsigned long long ref_pid_error_count(ref_pid_t *r) { return r->p.ERRORHandler.ERRORCount; }

/* ── FuzzyRule ── */
ref_fuzzy_t *ref_fuzzy_new(float kp_ratio, float ki_ratio, float kd_ratio, float e_step, float ec_step)
{
        ref_fuzzy_t *r = (ref_fuzzy_t *) calloc(1, sizeof(ref_fuzzy_t));
        if (!r) return NULL;
        Fuzzy_Rule_Init(&r->z, NULL, NULL, NULL, kp_ratio, ki_ratio, kd_ratio, e_step, ec_step);
        return r;
}
void ref_fuzzy_free(ref_fuzzy_t *r) { free(r); }
void ref_fuzzy_update(ref_fuzzy_t *r, float measure, float ref) { Fuzzy_Rule_Implementation(&r->z, measure, ref); }
void ref_pid_attach_fuzzy(ref_pid_t *p, ref_fuzzy_t *f) { p->p.FuzzyRule = &f->z; }

/* ── OLS ── */
ref_ols_t *ref_ols_new(unsigned short order)
{
        ref_ols_t *r = (ref_ols_t *) calloc(1, sizeof(ref_ols_t));
        if (!r) return NULL;
        OLS_Init(&r->o, order);
        return r;
}
void  ref_ols_free(ref_ols_t *r)
{
        if (!r) return;
        free(r->o.x);
        free(r->o.y);
        free(r);
}
void  ref_ols_update(ref_ols_t *r, float deltax, float y) { OLS_Update(&r->o, deltax, y); }
float ref_ols_derivative(ref_ols_t *r, float deltax, float y) { return OLS_Derivative(&r->o, deltax, y); }
float ref_ols_smooth(ref_ols_t *r, float deltax, float y) { return OLS_Smooth(&r->o, deltax, y); }
float ref_ols_k(ref_ols_t *r) { return r->o.k; }
float ref_ols_b(ref_ols_t *r) { return r->o.b; }

/* ── Feedforward ── */
ref_ff_t *ref_ff_new(float max_out, const float *c, float lpf_rc,
                     unsigned short dot_order, unsigned short ddot_order)
{
        ref_ff_t *r = (ref_ff_t *) calloc(1, sizeof(ref_ff_t));
        if (!r) return NULL;
        Feedforward_Init(&r->f, max_out, c, lpf_rc, dot_order, ddot_order);
        return r;
}
void  ref_ff_free(ref_ff_t *r) { free(r); }
float ref_ff_calc(ref_ff_t *r, float ref) { return Feedforward_Calculate(&r->f, ref); }
float ref_ff_output(ref_ff_t *r) { return r->f.Output; }

/* ── LDOB ── */
ref_ldob_t *ref_ldob_new(float max_d, float deadband, float *c, float lpf_rc,
                         unsigned short dot_order, unsigned short ddot_order)
{
        ref_ldob_t *r = (ref_ldob_t *) calloc(1, sizeof(ref_ldob_t));
        if (!r) return NULL;
        LDOB_Init(&r->l, max_d, deadband, c, lpf_rc, dot_order, ddot_order);
        return r;
}
void  ref_ldob_free(ref_ldob_t *r) { free(r); }
float ref_ldob_calc(ref_ldob_t *r, float measure, float u) { return LDOB_Calculate(&r->l, measure, u); }
float ref_ldob_output(ref_ldob_t *r) { return r->l.Output; }

/* ── TD ── */
ref_td_t *ref_td_new(float r, float h0)
{
        ref_td_t *t = (ref_td_t *) calloc(1, sizeof(ref_td_t));
        if (!t) return NULL;
        TD_Init(&t->t, r, h0);
        return t;
}
void  ref_td_free(ref_td_t *t) { free(t); }
float ref_td_calc(ref_td_t *t, float input) { return TD_Calculate(&t->t, input); }
float ref_td_x(ref_td_t *t) { return t->t.x; }

/* ── KalmanFilter ── */
ref_kf_t *ref_kf_new(unsigned char nx, unsigned char nu, unsigned char nz)
{
        ref_kf_t *r = (ref_kf_t *) calloc(1, sizeof(ref_kf_t));
        if (!r) return NULL;
        Kalman_Filter_Init(&r->k, nx, nu, nz);
        return r;
}
void ref_kf_free(ref_kf_t *r) { free(r); }
void ref_kf_set_F(ref_kf_t *r, const float *f) { memcpy(r->k.F_data, f, sizeof(float) * r->k.xhatSize * r->k.xhatSize); }
void ref_kf_set_H(ref_kf_t *r, const float *h) { memcpy(r->k.H_data, h, sizeof(float) * r->k.zSize * r->k.xhatSize); }
void ref_kf_set_Q(ref_kf_t *r, const float *q) { memcpy(r->k.Q_data, q, sizeof(float) * r->k.xhatSize * r->k.xhatSize); }
void ref_kf_set_Rdiag(ref_kf_t *r, const float *d) { memcpy(r->k.MatR_DiagonalElements, d, sizeof(float) * r->k.zSize); }
void ref_kf_set_x0(ref_kf_t *r, const float *x) { memcpy(r->k.xhat_data, x, sizeof(float) * r->k.xhatSize); }
void ref_kf_set_p0(ref_kf_t *r, const float *p) { memcpy(r->k.P_data, p, sizeof(float) * r->k.xhatSize * r->k.xhatSize); }
void ref_kf_set_measurement(ref_kf_t *r, unsigned char idx, float value) { if (idx < r->k.zSize) r->k.MeasuredVector[idx] = value; }
const float *ref_kf_update(ref_kf_t *r) { return Kalman_Filter_Update(&r->k); }
const float *ref_kf_state(ref_kf_t *r) { return r->k.xhat_data; }
const float *ref_kf_cov(ref_kf_t *r) { return r->k.P_data; }

/* ── CRC ── */
unsigned char  ref_crc8(const unsigned char *msg, unsigned int len, unsigned char init)
{
        return get_CRC8_check_sum((unsigned char *) msg, len, init);
}
unsigned short ref_crc16(const unsigned char *msg, unsigned int len, unsigned short init)
{
        return get_CRC16_check_sum((uint8_t *) msg, (uint32_t) len, init);
}
unsigned short ref_ccitt(unsigned short init, const unsigned char *msg, unsigned int len)
{
        return crc_ccitt(init, (uint8_t *) msg, (size_t) len);
}

/* ── QuaternionEKF（v2 用全局 QEKF_INS，只支持单实例）── */
ref_ekf_t *ref_ekf_new(float q1, float q2, float r, float lambda, float lpf)
{
        ref_ekf_t *e = (ref_ekf_t *) calloc(1, sizeof(ref_ekf_t));
        if (!e) return NULL;
        memset(&QEKF_INS, 0, sizeof(QEKF_INS));
        IMU_QuaternionEKF_Init(q1, q2, r, lambda, lpf);
        return e;
}
void ref_ekf_free(ref_ekf_t *e) { free(e); }
void ref_ekf_update(ref_ekf_t *e, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
        (void) e;
        IMU_QuaternionEKF_Update(gx, gy, gz, ax, ay, az, dt);
}
const float *ref_ekf_quat(ref_ekf_t *e) { (void) e; return QEKF_INS.q; }
const float *ref_ekf_bias(ref_ekf_t *e) { (void) e; return QEKF_INS.GyroBias; }
float ref_ekf_roll(ref_ekf_t *e) { (void) e; return QEKF_INS.Roll; }
float ref_ekf_pitch(ref_ekf_t *e) { (void) e; return QEKF_INS.Pitch; }
float ref_ekf_yaw(ref_ekf_t *e) { (void) e; return QEKF_INS.Yaw; }
unsigned char ref_ekf_converged(ref_ekf_t *e) { (void) e; return QEKF_INS.ConvergeFlag; }
