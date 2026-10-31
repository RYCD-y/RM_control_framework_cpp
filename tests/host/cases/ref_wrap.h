// v2 的 C 实现通过这层纯 C 接口暴露给 C++ 测试（避免 controller.h 的
// `struct pid_t` 与 POSIX 的 `pid_t` 冲突）
#ifndef REF_WRAP_H
#define REF_WRAP_H

#ifdef __cplusplus
extern "C" {
#endif

/* ── PID ── */
typedef struct ref_pid ref_pid_t;
ref_pid_t *ref_pid_new(float max_out, float integral_limit, float deadband,
                       float kp, float ki, float kd, float a, float b,
                       float output_lpf_rc, float derivative_lpf_rc,
                       unsigned short ols_order, unsigned char improve);
void  ref_pid_free(ref_pid_t *);
float ref_pid_calc(ref_pid_t *, float measure, float ref);
void  ref_pid_reset(ref_pid_t *);
void  ref_pid_set_gains(ref_pid_t *, float kp, float ki, float kd);
float ref_pid_output(ref_pid_t *);
float ref_pid_pout(ref_pid_t *);
float ref_pid_iout(ref_pid_t *);
float ref_pid_dout(ref_pid_t *);
float ref_pid_iterm(ref_pid_t *);
float ref_pid_err(ref_pid_t *);
float ref_pid_dt(ref_pid_t *);
unsigned long long ref_pid_error_count(ref_pid_t *);

/* ── FuzzyRule ── */
typedef struct ref_fuzzy ref_fuzzy_t;
ref_fuzzy_t *ref_fuzzy_new(float kp_ratio, float ki_ratio, float kd_ratio, float e_step, float ec_step);
void  ref_fuzzy_free(ref_fuzzy_t *);
void  ref_fuzzy_update(ref_fuzzy_t *, float measure, float ref);
void  ref_pid_attach_fuzzy(ref_pid_t *, ref_fuzzy_t *);

/* ── OLS ── */
typedef struct ref_ols ref_ols_t;
ref_ols_t *ref_ols_new(unsigned short order);
void  ref_ols_free(ref_ols_t *);
void  ref_ols_update(ref_ols_t *, float deltax, float y);
float ref_ols_derivative(ref_ols_t *, float deltax, float y);
float ref_ols_smooth(ref_ols_t *, float deltax, float y);
float ref_ols_k(ref_ols_t *);
float ref_ols_b(ref_ols_t *);

/* ── Feedforward ── */
typedef struct ref_ff ref_ff_t;
ref_ff_t *ref_ff_new(float max_out, const float *c, float lpf_rc,
                     unsigned short dot_order, unsigned short ddot_order);
void  ref_ff_free(ref_ff_t *);
float ref_ff_calc(ref_ff_t *, float ref);
float ref_ff_output(ref_ff_t *);

/* ── LDOB ── */
typedef struct ref_ldob ref_ldob_t;
ref_ldob_t *ref_ldob_new(float max_d, float deadband, float *c, float lpf_rc,
                         unsigned short dot_order, unsigned short ddot_order);
void  ref_ldob_free(ref_ldob_t *);
float ref_ldob_calc(ref_ldob_t *, float measure, float u);
float ref_ldob_output(ref_ldob_t *);

/* ── Tracking Differentiator ── */
typedef struct ref_td ref_td_t;
ref_td_t *ref_td_new(float r, float h0);
void  ref_td_free(ref_td_t *);
float ref_td_calc(ref_td_t *, float input);
float ref_td_x(ref_td_t *);

/* ── KalmanFilter ── */
typedef struct ref_kf ref_kf_t;
ref_kf_t   *ref_kf_new(unsigned char nx, unsigned char nu, unsigned char nz);
void        ref_kf_free(ref_kf_t *);
void        ref_kf_set_F(ref_kf_t *, const float *);
void        ref_kf_set_H(ref_kf_t *, const float *);
void        ref_kf_set_Q(ref_kf_t *, const float *);
void        ref_kf_set_Rdiag(ref_kf_t *, const float *);
void        ref_kf_set_x0(ref_kf_t *, const float *);
void        ref_kf_set_p0(ref_kf_t *, const float *);
void        ref_kf_set_measurement(ref_kf_t *, unsigned char idx, float value);
const float *ref_kf_update(ref_kf_t *);
const float *ref_kf_state(ref_kf_t *);
const float *ref_kf_cov(ref_kf_t *);

/* ── CRC8 / CRC16 ── */
unsigned char  ref_crc8(const unsigned char *msg, unsigned int len, unsigned char init);
unsigned short ref_crc16(const unsigned char *msg, unsigned int len, unsigned short init);
unsigned short ref_ccitt(unsigned short init, const unsigned char *msg, unsigned int len);

/* ── QuaternionEKF ── */
typedef struct ref_ekf ref_ekf_t;
ref_ekf_t   *ref_ekf_new(float q1, float q2, float r, float lambda, float lpf);
void         ref_ekf_free(ref_ekf_t *);
void         ref_ekf_update(ref_ekf_t *, float gx, float gy, float gz,
                            float ax, float ay, float az, float dt);
const float *ref_ekf_quat(ref_ekf_t *);
const float *ref_ekf_bias(ref_ekf_t *);
float        ref_ekf_roll(ref_ekf_t *);
float        ref_ekf_pitch(ref_ekf_t *);
float        ref_ekf_yaw(ref_ekf_t *);
unsigned char ref_ekf_converged(ref_ekf_t *);

#ifdef __cplusplus
}
#endif

#endif
