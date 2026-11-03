//
// 四元数姿态 EKF（陀螺零偏在线估计 + 卡方检验 + 渐消因子）
// 对应 control_framework_v2/UserCode/Common/QuaternionEKF.h + .c
//
#ifndef THREE_FRICTION_QUATERNION_EKF_HPP
#define THREE_FRICTION_QUATERNION_EKF_HPP

#include <stdint.h>

#include "kalman_filter.hpp"

class QuaternionEkf {
   public:
    QuaternionEkf() = default;

    // process_noise1: 四元数过程噪声(10)  process_noise2: 零偏过程噪声(0.001)
    // measure_noise : 加速度计量测噪声(1e7)  lambda: 渐消因子(0.9996)  lpf: 加速度低通系数(0)
    void Init(float process_noise1, float process_noise2, float measure_noise, float lambda,
              float lpf);

    void Update(float gx, float gy, float gz, float ax, float ay, float az, float Dt);

    // ── 只读查询 ──
    const float* Quaternion() const { return q_; }
    const float* gyroBias() const { return gyro_bias_; }
    const float* Gyro() const { return gyro_; }
    const float* Accel() const { return accel_; }
    float RollDeg() const { return roll_; }    // 度
    float PitchDeg() const { return pitch_; }  // 度
    float YawDeg() const { return yaw_; }      // 度
    float YawTotalAngleDeg() const { return yaw_total_angle_; }
    bool Converged() const { return converge_flag_ != 0; }
    bool stable() const { return stable_flag_ != 0; }
    bool Initialized() const { return initialized_ != 0; }
    uint64_t UpdateCount() const { return update_count_; }
    uint64_t ErrorCount() const { return error_count_; }
    float ChiSquare() const { return chi_square_data_[0]; }

   private:
    // KalmanFilter 的用户扩展环节（对应 v2 的 User_Func0..3）
    static void HookObserve(KalmanFilter& kf, void* ctx);
    static void HookFLinearizationPFading(KalmanFilter& kf, void* ctx);
    static void HookSetH(KalmanFilter& kf, void* ctx);
    static void HookXhatUpdate(KalmanFilter& kf, void* ctx);

    static float InvSqrt(float X);

    KalmanFilter kf_;

    float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro_bias_[3] = {};
    float gyro_[3] = {};
    float accel_[3] = {};
    float orientation_cosine_[3] = {};

    float acc_lpf_coef_ = 0.0f;
    float gyro_norm_ = 0.0f;
    float accl_norm_ = 0.0f;
    float adaptive_gain_scale_ = 1.0f;

    float roll_ = 0.0f;
    float pitch_ = 0.0f;
    float yaw_ = 0.0f;
    float yaw_total_angle_ = 0.0f;
    float yaw_angle_last_ = 0.0f;

    float q1_ = 0.0f;  // 四元数过程噪声
    float q2_ = 0.0f;  // 零偏过程噪声
    float r_ = 0.0f;   // 量测噪声
    float dt_ = 0.0f;
    float lambda_ = 1.0f;

    arm_matrix_instance_f32 chi_square_{};
    float chi_square_data_[1] = {};
    float chi_square_threshold_ = 1e-8f;

    int16_t yaw_round_count_ = 0;
    uint8_t initialized_ = 0;
    uint8_t converge_flag_ = 0;
    uint8_t stable_flag_ = 0;
    uint64_t error_count_ = 0;
    uint64_t update_count_ = 0;
};

#endif  // THREE_FRICTION_QUATERNION_EKF_HPP
