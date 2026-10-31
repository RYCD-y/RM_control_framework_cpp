//
// 卡尔曼滤波框架实现（逐行对应 v2 的 Kalman_Filter_*，矩阵用 arm_mat_*）
//
#include "kalman_filter.hpp"

#include <cstring>

namespace
{
template<typename T>
inline void zeroInit(T *&p, size_t count)
{
        p = static_cast<T *>(kfAlloc(sizeof(T) * count));
        if (p != nullptr) { std::memset(p, 0, sizeof(T) * count); }
}
} // namespace

KalmanFilter::~KalmanFilter() { releaseAll(); }

void KalmanFilter::releaseAll()
{
        // 所有缓冲区都来自 kfAlloc（pvPortMalloc / malloc）
        void **slots[] = {
                reinterpret_cast<void **>(&filtered_value_),   reinterpret_cast<void **>(&measured_vector_),
                reinterpret_cast<void **>(&control_vector_),   reinterpret_cast<void **>(&xhat_data_),
                reinterpret_cast<void **>(&xhatminus_data_),   reinterpret_cast<void **>(&u_data_),
                reinterpret_cast<void **>(&z_data_),           reinterpret_cast<void **>(&P_data_),
                reinterpret_cast<void **>(&Pminus_data_),      reinterpret_cast<void **>(&F_data_),
                reinterpret_cast<void **>(&FT_data_),          reinterpret_cast<void **>(&B_data_),
                reinterpret_cast<void **>(&H_data_),           reinterpret_cast<void **>(&HT_data_),
                reinterpret_cast<void **>(&Q_data_),           reinterpret_cast<void **>(&R_data_),
                reinterpret_cast<void **>(&K_data_),           reinterpret_cast<void **>(&S_data_),
                reinterpret_cast<void **>(&temp_matrix_data_), reinterpret_cast<void **>(&temp_matrix_data1_),
                reinterpret_cast<void **>(&temp_vector_data_), reinterpret_cast<void **>(&temp_vector_data1_),
                reinterpret_cast<void **>(&measurement_map_),  reinterpret_cast<void **>(&temp_),
                reinterpret_cast<void **>(&measurement_degree_), reinterpret_cast<void **>(&r_diagonal_),
                reinterpret_cast<void **>(&state_min_variance_),
        };
        for (void **slot : slots)
        {
                if (*slot != nullptr) { free(*slot); *slot = nullptr; }
        }
}

void KalmanFilter::init(uint8_t xhat_size, uint8_t u_size, uint8_t z_size)
{
        releaseAll();

        xhat_size_ = xhat_size;
        u_size_    = u_size;
        z_size_    = z_size;

        measurement_valid_num_ = 0;

        zeroInit(measurement_map_, z_size);
        zeroInit(measurement_degree_, z_size);
        zeroInit(r_diagonal_, z_size);
        zeroInit(state_min_variance_, xhat_size);
        zeroInit(temp_, z_size);

        zeroInit(filtered_value_, xhat_size);
        zeroInit(measured_vector_, z_size);
        zeroInit(control_vector_, u_size);

        zeroInit(xhat_data_, xhat_size);
        arm_mat_init_f32(&xhat_, xhat_size_, 1, xhat_data_);

        zeroInit(xhatminus_data_, xhat_size);
        arm_mat_init_f32(&xhatminus_, xhat_size_, 1, xhatminus_data_);

        if (u_size != 0U)
        {
                zeroInit(u_data_, u_size);
                arm_mat_init_f32(&u_, u_size_, 1, u_data_);
        }

        zeroInit(z_data_, z_size);
        arm_mat_init_f32(&z_, z_size_, 1, z_data_);

        zeroInit(P_data_, static_cast<size_t>(xhat_size) * xhat_size);
        arm_mat_init_f32(&P_, xhat_size_, xhat_size_, P_data_);

        zeroInit(Pminus_data_, static_cast<size_t>(xhat_size) * xhat_size);
        arm_mat_init_f32(&Pminus_, xhat_size_, xhat_size_, Pminus_data_);

        zeroInit(F_data_, static_cast<size_t>(xhat_size) * xhat_size);
        zeroInit(FT_data_, static_cast<size_t>(xhat_size) * xhat_size);
        arm_mat_init_f32(&F_, xhat_size_, xhat_size_, F_data_);
        arm_mat_init_f32(&FT_, xhat_size_, xhat_size_, FT_data_);

        if (u_size != 0U)
        {
                zeroInit(B_data_, static_cast<size_t>(xhat_size) * u_size);
                arm_mat_init_f32(&B_, xhat_size_, u_size_, B_data_);
        }

        zeroInit(H_data_, static_cast<size_t>(z_size) * xhat_size);
        zeroInit(HT_data_, static_cast<size_t>(xhat_size) * z_size);
        arm_mat_init_f32(&H_, z_size_, xhat_size_, H_data_);
        arm_mat_init_f32(&HT_, xhat_size_, z_size_, HT_data_);

        zeroInit(Q_data_, static_cast<size_t>(xhat_size) * xhat_size);
        arm_mat_init_f32(&Q_, xhat_size_, xhat_size_, Q_data_);

        zeroInit(R_data_, static_cast<size_t>(z_size) * z_size);
        arm_mat_init_f32(&R_, z_size_, z_size_, R_data_);

        zeroInit(K_data_, static_cast<size_t>(xhat_size) * z_size);
        arm_mat_init_f32(&K_, xhat_size_, z_size_, K_data_);

        zeroInit(S_data_, static_cast<size_t>(xhat_size) * xhat_size);
        zeroInit(temp_matrix_data_, static_cast<size_t>(xhat_size) * xhat_size);
        zeroInit(temp_matrix_data1_, static_cast<size_t>(xhat_size) * xhat_size);
        zeroInit(temp_vector_data_, xhat_size);
        zeroInit(temp_vector_data1_, xhat_size);
        arm_mat_init_f32(&S_, xhat_size_, xhat_size_, S_data_);
        arm_mat_init_f32(&temp_matrix_, xhat_size_, xhat_size_, temp_matrix_data_);
        arm_mat_init_f32(&temp_matrix1_, xhat_size_, xhat_size_, temp_matrix_data1_);
        arm_mat_init_f32(&temp_vector_, xhat_size_, 1, temp_vector_data_);
        arm_mat_init_f32(&temp_vector1_, xhat_size_, 1, temp_vector_data1_);

        skip_eq1_ = false;
        skip_eq2_ = false;
        skip_eq3_ = false;
        skip_eq4_ = false;
        skip_eq5_ = false;
}

void KalmanFilter::setStateTransition(const float *f)
{
        std::memcpy(F_data_, f, sizeof(float) * xhat_size_ * xhat_size_);
}

void KalmanFilter::setControlMatrix(const float *b)
{
        if (u_size_ == 0U || B_data_ == nullptr) { return; }
        std::memcpy(B_data_, b, sizeof(float) * xhat_size_ * u_size_);
}

void KalmanFilter::setMeasurementMatrix(const float *h)
{
        std::memcpy(H_data_, h, sizeof(float) * z_size_ * xhat_size_);
}

void KalmanFilter::setMeasurementMap(const uint8_t *map)
{
        std::memcpy(measurement_map_, map, sizeof(uint8_t) * z_size_);
}

void KalmanFilter::setMeasurementDegree(const float *degree)
{
        std::memcpy(measurement_degree_, degree, sizeof(float) * z_size_);
}

void KalmanFilter::setProcessNoise(const float *q)
{
        std::memcpy(Q_data_, q, sizeof(float) * xhat_size_ * xhat_size_);
}

void KalmanFilter::setMeasurementNoiseDiag(const float *r_diag)
{
        std::memcpy(r_diagonal_, r_diag, sizeof(float) * z_size_);
}

void KalmanFilter::setStateMinVariance(const float *min_var)
{
        std::memcpy(state_min_variance_, min_var, sizeof(float) * xhat_size_);
}

void KalmanFilter::setInitialState(const float *x0)
{
        std::memcpy(xhat_data_, x0, sizeof(float) * xhat_size_);
}

void KalmanFilter::setInitialCovariance(const float *p0)
{
        std::memcpy(P_data_, p0, sizeof(float) * xhat_size_ * xhat_size_);
}

void KalmanFilter::setMeasurement(uint8_t idx, float value)
{
        if (idx < z_size_ && measured_vector_ != nullptr) { measured_vector_[idx] = value; }
}

void KalmanFilter::setControl(uint8_t idx, float value)
{
        if (idx < u_size_ && control_vector_ != nullptr) { control_vector_[idx] = value; }
}

float KalmanFilter::measurement(uint8_t idx) const
{
        if (idx < z_size_ && measured_vector_ != nullptr) { return measured_vector_[idx]; }
        return 0.0f;
}

void KalmanFilter::clearMeasurements()
{
        if (measured_vector_ != nullptr) { std::memset(measured_vector_, 0, sizeof(float) * z_size_); }
}

float KalmanFilter::filteredValue(uint8_t i) const
{
        if (i < xhat_size_ && filtered_value_ != nullptr) { return filtered_value_[i]; }
        return 0.0f;
}

void KalmanFilter::setSkipFlags(bool eq1, bool eq2, bool eq3, bool eq4, bool eq5)
{
        skip_eq1_ = eq1;
        skip_eq2_ = eq2;
        skip_eq3_ = eq3;
        skip_eq4_ = eq4;
        skip_eq5_ = eq5;
}

void KalmanFilter::setHooks(Hook f0, Hook f1, Hook f2, Hook f3, Hook f4, Hook f5, Hook f6, void *ctx)
{
        func0_ = f0;
        func1_ = f1;
        func2_ = f2;
        func3_ = f3;
        func4_ = f4;
        func5_ = f5;
        func6_ = f6;
        hook_ctx_ = ctx;
}

void KalmanFilter::hkrAdjustment()
{
        measurement_valid_num_ = 0;

        std::memcpy(z_data_, measured_vector_, sizeof(float) * z_size_);
        std::memset(measured_vector_, 0, sizeof(float) * z_size_);

        std::memset(R_data_, 0, sizeof(float) * z_size_ * z_size_);
        std::memset(H_data_, 0, sizeof(float) * xhat_size_ * z_size_);

        for (uint8_t i = 0; i < z_size_; i++)
        {
                if (z_data_[i] != 0.0f)
                {
                        z_data_[measurement_valid_num_] = z_data_[i];
                        temp_[measurement_valid_num_]   = i;
                        H_data_[static_cast<size_t>(xhat_size_) * measurement_valid_num_ +
                                measurement_map_[i] - 1U] = measurement_degree_[i];
                        measurement_valid_num_++;
                }
        }

        for (uint8_t i = 0; i < measurement_valid_num_; i++)
        {
                R_data_[static_cast<size_t>(i) * measurement_valid_num_ + i] = r_diagonal_[temp_[i]];
        }

        H_.numRows  = measurement_valid_num_;
        H_.numCols  = xhat_size_;
        HT_.numRows = xhat_size_;
        HT_.numCols = measurement_valid_num_;
        R_.numRows  = measurement_valid_num_;
        R_.numCols  = measurement_valid_num_;
        K_.numRows  = xhat_size_;
        K_.numCols  = measurement_valid_num_;
        z_.numRows  = measurement_valid_num_;
}

void KalmanFilter::measure()
{
        if (use_auto_adjustment_) { hkrAdjustment(); }
        else
        {
                std::memcpy(z_data_, measured_vector_, sizeof(float) * z_size_);
                std::memset(measured_vector_, 0, sizeof(float) * z_size_);
        }
        std::memcpy(u_data_, control_vector_, sizeof(float) * u_size_);
}

void KalmanFilter::xhatMinusUpdate()
{
        if (skip_eq1_) { return; }

        if (u_size_ > 0U)
        {
                temp_vector_.numRows = xhat_size_;
                temp_vector_.numCols = 1;
                mat_status_ = arm_mat_mult_f32(&F_, &xhat_, &temp_vector_);
                temp_vector1_.numRows = xhat_size_;
                temp_vector1_.numCols = 1;
                mat_status_ = arm_mat_mult_f32(&B_, &u_, &temp_vector1_);
                mat_status_ = arm_mat_add_f32(&temp_vector_, &temp_vector1_, &xhatminus_);
        }
        else
        {
                mat_status_ = arm_mat_mult_f32(&F_, &xhat_, &xhatminus_);
        }
}

void KalmanFilter::pMinusUpdate()
{
        if (skip_eq2_) { return; }

        mat_status_ = arm_mat_trans_f32(&F_, &FT_);
        mat_status_ = arm_mat_mult_f32(&F_, &P_, &Pminus_);
        temp_matrix_.numRows = Pminus_.numRows;
        temp_matrix_.numCols = FT_.numCols;
        mat_status_ = arm_mat_mult_f32(&Pminus_, &FT_, &temp_matrix_);
        mat_status_ = arm_mat_add_f32(&temp_matrix_, &Q_, &Pminus_);
}

void KalmanFilter::setK()
{
        if (skip_eq3_) { return; }

        mat_status_ = arm_mat_trans_f32(&H_, &HT_);
        temp_matrix_.numRows = H_.numRows;
        temp_matrix_.numCols = Pminus_.numCols;
        mat_status_ = arm_mat_mult_f32(&H_, &Pminus_, &temp_matrix_);
        temp_matrix1_.numRows = temp_matrix_.numRows;
        temp_matrix1_.numCols = HT_.numCols;
        mat_status_ = arm_mat_mult_f32(&temp_matrix_, &HT_, &temp_matrix1_);
        S_.numRows = R_.numRows;
        S_.numCols = R_.numCols;
        mat_status_ = arm_mat_add_f32(&temp_matrix1_, &R_, &S_);
        mat_status_ = arm_mat_inverse_f32(&S_, &temp_matrix1_);
        temp_matrix_.numRows = Pminus_.numRows;
        temp_matrix_.numCols = HT_.numCols;
        mat_status_ = arm_mat_mult_f32(&Pminus_, &HT_, &temp_matrix_);
        mat_status_ = arm_mat_mult_f32(&temp_matrix_, &temp_matrix1_, &K_);
}

void KalmanFilter::xhatUpdate()
{
        if (skip_eq4_) { return; }

        temp_vector_.numRows = H_.numRows;
        temp_vector_.numCols = 1;
        mat_status_ = arm_mat_mult_f32(&H_, &xhatminus_, &temp_vector_);
        temp_vector1_.numRows = z_.numRows;
        temp_vector1_.numCols = 1;
        mat_status_ = arm_mat_sub_f32(&z_, &temp_vector_, &temp_vector1_);
        temp_vector_.numRows = K_.numRows;
        temp_vector_.numCols = 1;
        mat_status_ = arm_mat_mult_f32(&K_, &temp_vector1_, &temp_vector_);
        mat_status_ = arm_mat_add_f32(&xhatminus_, &temp_vector_, &xhat_);
}

void KalmanFilter::pUpdate()
{
        if (skip_eq5_) { return; }

        temp_matrix_.numRows = K_.numRows;
        temp_matrix_.numCols = H_.numCols;
        temp_matrix1_.numRows = temp_matrix_.numRows;
        temp_matrix1_.numCols = Pminus_.numCols;
        mat_status_ = arm_mat_mult_f32(&K_, &H_, &temp_matrix_);
        mat_status_ = arm_mat_mult_f32(&temp_matrix_, &Pminus_, &temp_matrix1_);
        mat_status_ = arm_mat_sub_f32(&Pminus_, &temp_matrix1_, &P_);
}

const float *KalmanFilter::update()
{
        // 0. 获取量测
        measure();
        if (func0_ != nullptr) { func0_(*this, hook_ctx_); }

        // 1. xhat(k|k-1) = F·xhat(k-1) + B·u
        xhatMinusUpdate();
        if (func1_ != nullptr) { func1_(*this, hook_ctx_); }

        // 2. P(k|k-1) = F·P(k-1)·FT + Q
        pMinusUpdate();
        if (func2_ != nullptr) { func2_(*this, hook_ctx_); }

        if (measurement_valid_num_ != 0U || !use_auto_adjustment_)
        {
                // 3. K = P'·HT / (H·P'·HT + R)
                setK();
                if (func3_ != nullptr) { func3_(*this, hook_ctx_); }

                // 4. xhat(k) = xhat'(k) + K·(z - H·xhat'(k))
                xhatUpdate();
                if (func4_ != nullptr) { func4_(*this, hook_ctx_); }

                // 5. P(k) = P'(k) - K·H·P'(k)
                pUpdate();
        }
        else
        {
                std::memcpy(xhat_data_, xhatminus_data_, sizeof(float) * xhat_size_);
                std::memcpy(P_data_, Pminus_data_, sizeof(float) * xhat_size_ * xhat_size_);
        }

        if (func5_ != nullptr) { func5_(*this, hook_ctx_); }

        // 抑制过度收敛
        for (uint8_t i = 0; i < xhat_size_; i++)
        {
                if (P_data_[static_cast<size_t>(i) * xhat_size_ + i] < state_min_variance_[i])
                {
                        P_data_[static_cast<size_t>(i) * xhat_size_ + i] = state_min_variance_[i];
                }
        }

        std::memcpy(filtered_value_, xhat_data_, sizeof(float) * xhat_size_);

        if (func6_ != nullptr) { func6_(*this, hook_ctx_); }

        return filtered_value_;
}
