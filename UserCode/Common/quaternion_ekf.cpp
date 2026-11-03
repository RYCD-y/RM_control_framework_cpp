//
// 四元数姿态 EKF 实现（逐行对应 v2 的 QuaternionEKF.c）
//
#include "quaternion_ekf.hpp"

#include <cmath>
#include <cstring>

namespace {
// 单位阵与初始协方差（v2: IMU_QuaternionEKF_F / IMU_QuaternionEKF_P）
constexpr float kInitF[36] = {
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
};

constexpr float kInitP[36] = {
    100000, 0.1, 0.1,    0.1, 0.1, 0.1, 0.1, 100000, 0.1, 0.1,    0.1, 0.1,
    0.1,    0.1, 100000, 0.1, 0.1, 0.1, 0.1, 0.1,    0.1, 100000, 0.1, 0.1,
    0.1,    0.1, 0.1,    0.1, 100, 0.1, 0.1, 0.1,    0.1, 0.1,    0.1, 100,
};
}  // namespace

float QuaternionEkf::InvSqrt(float X) {
    const float halfx = 0.5f * X;
    float y = X;
    int32_t i = 0;
    std::memcpy(&i, &y, sizeof(i));
    i = 0x5f375a86 - (i >> 1);
    std::memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void QuaternionEkf::Init(float process_noise1, float process_noise2, float measure_noise,
                         float lambda, float lpf) {
    initialized_ = 1;
    q1_ = process_noise1;
    q2_ = process_noise2;
    r_ = measure_noise;

    chi_square_threshold_ = 1e-8f;
    converge_flag_ = 0;
    error_count_ = 0;
    update_count_ = 0;

    lambda_ = (lambda > 1.0f) ? 1.0f : lambda;
    acc_lpf_coef_ = lpf;

    kf_.Init(6, 0, 3);
    arm_mat_init_f32(&chi_square_, 1, 1, chi_square_data_);

    // 姿态初始化：q = [1,0,0,0]
    const float x0[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    kf_.SetInitialState(x0);

    kf_.SetHooks(&QuaternionEkf::HookObserve, &QuaternionEkf::HookFLinearizationPFading,
                 &QuaternionEkf::HookSetH, &QuaternionEkf::HookXhatUpdate, nullptr, nullptr,
                 nullptr, this);

    // 用自定义环节替换 K 计算与后验融合
    kf_.SetSkipFlags(false, false, true, true, false);

    kf_.SetStateTransition(kInitF);
    kf_.SetInitialCovariance(kInitP);
}

void QuaternionEkf::Update(float gx, float gy, float gz, float ax, float ay, float az, float Dt) {
    if (initialized_ == 0) {
        Init(10.0f, 0.001f, 1000000.0f * 10.0f, 1.0f, 0.0f);
    }

    dt_ = Dt;

    gyro_[0] = gx - gyro_bias_[0];
    gyro_[1] = gy - gyro_bias_[1];
    gyro_[2] = gz - gyro_bias_[2];

    const float half_gx_dt = 0.5f * gyro_[0] * Dt;
    const float half_gy_dt = 0.5f * gyro_[1] * Dt;
    const float half_gz_dt = 0.5f * gyro_[2] * Dt;

    std::memcpy(kf_.F_data_, kInitF, sizeof(kInitF));

    kf_.F_data_[1] = -half_gx_dt;
    kf_.F_data_[2] = -half_gy_dt;
    kf_.F_data_[3] = -half_gz_dt;

    kf_.F_data_[6] = half_gx_dt;
    kf_.F_data_[8] = half_gz_dt;
    kf_.F_data_[9] = -half_gy_dt;

    kf_.F_data_[12] = half_gy_dt;
    kf_.F_data_[13] = -half_gz_dt;
    kf_.F_data_[15] = half_gx_dt;

    kf_.F_data_[18] = half_gz_dt;
    kf_.F_data_[19] = half_gy_dt;
    kf_.F_data_[20] = -half_gx_dt;

    // 加速度低通
    if (update_count_ == 0) {
        accel_[0] = ax;
        accel_[1] = ay;
        accel_[2] = az;
    }
    accel_[0] =
        accel_[0] * acc_lpf_coef_ / (dt_ + acc_lpf_coef_) + ax * dt_ / (dt_ + acc_lpf_coef_);
    accel_[1] =
        accel_[1] * acc_lpf_coef_ / (dt_ + acc_lpf_coef_) + ay * dt_ / (dt_ + acc_lpf_coef_);
    accel_[2] =
        accel_[2] * acc_lpf_coef_ / (dt_ + acc_lpf_coef_) + az * dt_ / (dt_ + acc_lpf_coef_);

    // 量测：单位化重力方向
    const float accel_inv_norm =
        InvSqrt(accel_[0] * accel_[0] + accel_[1] * accel_[1] + accel_[2] * accel_[2]);
    for (uint8_t i = 0; i < 3U; i++) {
        kf_.measured_vector_[i] = accel_[i] * accel_inv_norm;
    }

    gyro_norm_ = 1.0f / InvSqrt(gyro_[0] * gyro_[0] + gyro_[1] * gyro_[1] + gyro_[2] * gyro_[2]);
    accl_norm_ = 1.0f / accel_inv_norm;

    if (gyro_norm_ < 0.3f && accl_norm_ > 9.8f - 0.5f && accl_norm_ < 9.8f + 0.5f) {
        stable_flag_ = 1;
    } else {
        stable_flag_ = 0;
    }

    kf_.Q_data_[0] = q1_ * dt_;
    kf_.Q_data_[7] = q1_ * dt_;
    kf_.Q_data_[14] = q1_ * dt_;
    kf_.Q_data_[21] = q1_ * dt_;
    kf_.Q_data_[28] = q2_ * dt_;
    kf_.Q_data_[35] = q2_ * dt_;

    kf_.R_data_[0] = r_;
    kf_.R_data_[4] = r_;
    kf_.R_data_[8] = r_;

    (void)kf_.Update();

    q_[0] = kf_.filtered_value_[0];
    q_[1] = kf_.filtered_value_[1];
    q_[2] = kf_.filtered_value_[2];
    q_[3] = kf_.filtered_value_[3];

    gyro_bias_[0] = kf_.filtered_value_[4];
    gyro_bias_[1] = kf_.filtered_value_[5];
    gyro_bias_[2] = 0.0f;  // z 轴不可观测

    yaw_ = std::atan2(2.0f * (q_[0] * q_[3] + q_[1] * q_[2]),
                      2.0f * (q_[0] * q_[0] + q_[1] * q_[1]) - 1.0f) *
           57.295779513f;
    pitch_ = std::atan2(2.0f * (q_[0] * q_[1] + q_[2] * q_[3]),
                        2.0f * (q_[0] * q_[0] + q_[3] * q_[3]) - 1.0f) *
             57.295779513f;
    roll_ = std::asin(-2.0f * (q_[1] * q_[3] - q_[0] * q_[2])) * 57.295779513f;

    if (yaw_ - yaw_angle_last_ > 180.0f) {
        yaw_round_count_--;
    } else if (yaw_ - yaw_angle_last_ < -180.0f) {
        yaw_round_count_++;
    }

    yaw_total_angle_ = 360.0f * static_cast<float>(yaw_round_count_) + yaw_;
    yaw_angle_last_ = yaw_;
    update_count_++;
}

/* ═══ KalmanFilter 钩子 ═══ */
void QuaternionEkf::HookObserve(KalmanFilter& kf, void* ctx) {
    (void)kf;
    (void)ctx;  // v2 这里只是把 P/K/H 快照到全局数组供观察；C++ 版无此需求
}

void QuaternionEkf::HookFLinearizationPFading(KalmanFilter& kf, void* ctx) {
    QuaternionEkf& self = *static_cast<QuaternionEkf*>(ctx);

    const float q0 = kf.xhatminus_data_[0];
    const float q1 = kf.xhatminus_data_[1];
    const float q2 = kf.xhatminus_data_[2];
    const float q3 = kf.xhatminus_data_[3];

    const float q_inv_norm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4U; i++) {
        kf.xhatminus_data_[i] *= q_inv_norm;
    }

    // 状态转移矩阵右上角 4x2 子矩阵（与陀螺零偏的耦合）
    kf.F_data_[4] = q1 * self.dt_ / 2.0f;
    kf.F_data_[5] = q2 * self.dt_ / 2.0f;
    kf.F_data_[10] = -q0 * self.dt_ / 2.0f;
    kf.F_data_[11] = q3 * self.dt_ / 2.0f;
    kf.F_data_[16] = -q3 * self.dt_ / 2.0f;
    kf.F_data_[17] = -q0 * self.dt_ / 2.0f;
    kf.F_data_[22] = q2 * self.dt_ / 2.0f;
    kf.F_data_[23] = -q1 * self.dt_ / 2.0f;

    // 渐消因子，抑制零偏过度收敛
    kf.P_data_[28] /= self.lambda_;
    kf.P_data_[35] /= self.lambda_;

    if (kf.P_data_[28] > 10000.0f) {
        kf.P_data_[28] = 10000.0f;
    }
    if (kf.P_data_[35] > 10000.0f) {
        kf.P_data_[35] = 10000.0f;
    }
}

void QuaternionEkf::HookSetH(KalmanFilter& kf, void* ctx) {
    (void)ctx;
    const float double_q0 = 2.0f * kf.xhatminus_data_[0];
    const float double_q1 = 2.0f * kf.xhatminus_data_[1];
    const float double_q2 = 2.0f * kf.xhatminus_data_[2];
    const float double_q3 = 2.0f * kf.xhatminus_data_[3];

    std::memset(kf.H_data_, 0, sizeof(float) * kf.z_size_ * kf.xhat_size_);

    kf.H_data_[0] = -double_q2;
    kf.H_data_[1] = double_q3;
    kf.H_data_[2] = -double_q0;
    kf.H_data_[3] = double_q1;

    kf.H_data_[6] = double_q1;
    kf.H_data_[7] = double_q0;
    kf.H_data_[8] = double_q3;
    kf.H_data_[9] = double_q2;

    kf.H_data_[12] = double_q0;
    kf.H_data_[13] = -double_q1;
    kf.H_data_[14] = -double_q2;
    kf.H_data_[15] = double_q3;
}

void QuaternionEkf::HookXhatUpdate(KalmanFilter& kf, void* ctx) {
    QuaternionEkf& self = *static_cast<QuaternionEkf*>(ctx);

    // S = H·P'·HT + R，再求逆
    kf.mat_status_ = arm_mat_trans_f32(&kf.H_, &kf.HT_);
    kf.temp_matrix_.numRows = kf.H_.numRows;
    kf.temp_matrix_.numCols = kf.Pminus_.numCols;
    kf.mat_status_ = arm_mat_mult_f32(&kf.H_, &kf.Pminus_, &kf.temp_matrix_);
    kf.temp_matrix1_.numRows = kf.temp_matrix_.numRows;
    kf.temp_matrix1_.numCols = kf.HT_.numCols;
    kf.mat_status_ = arm_mat_mult_f32(&kf.temp_matrix_, &kf.HT_, &kf.temp_matrix1_);
    kf.S_.numRows = kf.R_.numRows;
    kf.S_.numCols = kf.R_.numCols;
    kf.mat_status_ = arm_mat_add_f32(&kf.temp_matrix1_, &kf.R_, &kf.S_);
    kf.mat_status_ = arm_mat_inverse_f32(&kf.S_, &kf.temp_matrix1_);

    const float q0 = kf.xhatminus_data_[0];
    const float q1 = kf.xhatminus_data_[1];
    const float q2 = kf.xhatminus_data_[2];
    const float q3 = kf.xhatminus_data_[3];

    kf.temp_vector_.numRows = kf.H_.numRows;
    kf.temp_vector_.numCols = 1;
    kf.temp_vector_data_[0] = 2.0f * (q1 * q3 - q0 * q2);
    kf.temp_vector_data_[1] = 2.0f * (q0 * q1 + q2 * q3);
    kf.temp_vector_data_[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    for (uint8_t i = 0; i < 3U; i++) {
        self.orientation_cosine_[i] = std::acos(std::fabs(kf.temp_vector_data_[i]));
    }

    kf.temp_vector1_.numRows = kf.z_.numRows;
    kf.temp_vector1_.numCols = 1;
    kf.mat_status_ = arm_mat_sub_f32(&kf.z_, &kf.temp_vector_, &kf.temp_vector1_);

    // 卡方检验
    kf.temp_matrix_.numRows = kf.temp_vector1_.numRows;
    kf.temp_matrix_.numCols = 1;
    kf.mat_status_ = arm_mat_mult_f32(&kf.temp_matrix1_, &kf.temp_vector1_, &kf.temp_matrix_);
    kf.temp_vector_.numRows = 1;
    kf.temp_vector_.numCols = kf.temp_vector1_.numRows;
    kf.mat_status_ = arm_mat_trans_f32(&kf.temp_vector1_, &kf.temp_vector_);
    kf.mat_status_ = arm_mat_mult_f32(&kf.temp_vector_, &kf.temp_matrix_, &self.chi_square_);

    if (self.chi_square_data_[0] < 0.5f * self.chi_square_threshold_) {
        self.converge_flag_ = 1;
    }

    if (self.chi_square_data_[0] > self.chi_square_threshold_ && self.converge_flag_ != 0) {
        if (self.stable_flag_ != 0) {
            self.error_count_++;
        } else {
            self.error_count_ = 0;
        }

        if (self.error_count_ > 50U) {
            self.converge_flag_ = 0;
            kf.skip_eq5_ = false;
        } else {
            std::memcpy(kf.xhat_data_, kf.xhatminus_data_, sizeof(float) * kf.xhat_size_);
            std::memcpy(kf.P_data_, kf.Pminus_data_, sizeof(float) * kf.xhat_size_ * kf.xhat_size_);
            kf.skip_eq5_ = true;
            return;
        }
    } else {
        if (self.chi_square_data_[0] > 0.1f * self.chi_square_threshold_ &&
            self.converge_flag_ != 0) {
            self.adaptive_gain_scale_ = (self.chi_square_threshold_ - self.chi_square_data_[0]) /
                                        (0.9f * self.chi_square_threshold_);
        } else {
            self.adaptive_gain_scale_ = 1.0f;
        }
        self.error_count_ = 0;
        kf.skip_eq5_ = false;
    }

    // K = P'·HT·inv(S)
    kf.temp_matrix_.numRows = kf.Pminus_.numRows;
    kf.temp_matrix_.numCols = kf.HT_.numCols;
    kf.mat_status_ = arm_mat_mult_f32(&kf.Pminus_, &kf.HT_, &kf.temp_matrix_);
    kf.mat_status_ = arm_mat_mult_f32(&kf.temp_matrix_, &kf.temp_matrix1_, &kf.K_);

    for (uint8_t i = 0; i < kf.K_.numRows * kf.K_.numCols; i++) {
        kf.K_data_[i] *= self.adaptive_gain_scale_;
    }
    for (uint8_t i = 4; i < 6U; i++) {
        for (uint8_t j = 0; j < 3U; j++) {
            kf.K_data_[i * 3U + j] *= self.orientation_cosine_[i - 4U] / 1.5707963f;
        }
    }

    kf.temp_vector_.numRows = kf.K_.numRows;
    kf.temp_vector_.numCols = 1;
    kf.mat_status_ = arm_mat_mult_f32(&kf.K_, &kf.temp_vector1_, &kf.temp_vector_);

    // 零漂修正限幅
    if (self.converge_flag_ != 0) {
        for (uint8_t i = 4; i < 6U; i++) {
            if (kf.temp_vector_.pData[i] > 1e-2f * self.dt_) {
                kf.temp_vector_.pData[i] = 1e-2f * self.dt_;
            }
            if (kf.temp_vector_.pData[i] < -1e-2f * self.dt_) {
                kf.temp_vector_.pData[i] = -1e-2f * self.dt_;
            }
        }
    }

    // 不修正 YawDeg 轴
    kf.temp_vector_.pData[3] = 0.0f;
    kf.mat_status_ = arm_mat_add_f32(&kf.xhatminus_, &kf.temp_vector_, &kf.xhat_);
}
