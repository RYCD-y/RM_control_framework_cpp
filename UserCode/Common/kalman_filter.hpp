//
// 通用卡尔曼滤波框架（状态/控制/量测维度运行期指定，矩阵用 CMSIS-DSP）
// 对应 control_framework_v2/UserCode/Common/kalman_filter.h + .c
//
#ifndef THREE_FRICTION_KALMAN_FILTER_HPP
#define THREE_FRICTION_KALMAN_FILTER_HPP

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "arm_math.h"

#if defined(HOST_TEST)
inline void* kfAlloc(size_t n) { return malloc(n); }
#else
#include "FreeRTOS.h"
inline void* kfAlloc(size_t n) { return pvPortMalloc(n); }
#endif

class KalmanFilter {
   public:
    /// 用户自定义环节（对应 v2 的 User_Func0..6）
    using Hook = void (*)(KalmanFilter&, void* ctx);

    KalmanFilter() = default;
    ~KalmanFilter();

    KalmanFilter(const KalmanFilter&) = delete;
    KalmanFilter& operator=(const KalmanFilter&) = delete;

    void Init(uint8_t xhat_size, uint8_t u_size, uint8_t z_size);

    // ── 语义化注入（不暴露裸矩阵指针）──
    void SetStateTransition(const float* f);            // xhatSize²
    void SetControlMatrix(const float* b);              // XhatSize×USize
    void SetMeasurementMatrix(const float* h);          // ZSize×XhatSize
    void SetMeasurementMap(const uint8_t* map);         // ZSize（1..XhatSize）
    void SetMeasurementDegree(const float* degree);     // ZSize
    void SetProcessNoise(const float* q);               // xhatSize²
    void SetMeasurementNoiseDiag(const float* r_diag);  // ZSize
    void SetStateMinVariance(const float* min_var);     // XhatSize
    void SetInitialState(const float* x0);              // XhatSize
    void SetInitialCovariance(const float* p0);         // xhatSize²

    void SetMeasurement(uint8_t idx, float value);  // 写 MeasuredVector[idx]
    void SetControl(uint8_t idx, float value);      // 写 ControlVector[idx]
    float Measurement(uint8_t idx) const;
    void ClearMeasurements();

    void SetUseAutoAdjustment(bool on) { use_auto_adjustment_ = on; }
    uint8_t MeasurementValidNum() const { return measurement_valid_num_; }
    int8_t MatStatus() const { return mat_status_; }

    void SetSkipFlags(bool eq1, bool eq2, bool eq3, bool eq4, bool eq5);
    void SetHooks(Hook f0, Hook f1, Hook f2, Hook f3, Hook f4, Hook f5, Hook f6,
                  void* ctx = nullptr);

    // ── 计算 ──
    void Measure();
    void XhatMinusUpdate();
    void PMinusUpdate();
    void SetK();
    void XhatUpdate();
    void PUpdate();
    const float* Update();

    // ── 只读查询 ──
    const float* State() const { return xhat_data_; }
    const float* Covariance() const { return P_data_; }
    float FilteredValue(uint8_t i) const;
    uint8_t XhatSize() const { return xhat_size_; }
    uint8_t USize() const { return u_size_; }
    uint8_t ZSize() const { return z_size_; }

    // EKF 需要每帧改写 F/H/Q/R 的内部元素
    friend class QuaternionEkf;

   protected:
    void HkrAdjustment();
    void ReleaseAll();

    arm_matrix_instance_f32 xhat_{}, xhatminus_{}, u_{}, z_{};
    arm_matrix_instance_f32 P_{}, Pminus_{}, F_{}, FT_{}, B_{};
    arm_matrix_instance_f32 H_{}, HT_{}, Q_{}, R_{}, K_{}, S_{};
    arm_matrix_instance_f32 temp_matrix_{}, temp_matrix1_{};
    arm_matrix_instance_f32 temp_vector_{}, temp_vector1_{};

    float* filtered_value_ = nullptr;
    float* measured_vector_ = nullptr;
    float* control_vector_ = nullptr;
    float* xhat_data_ = nullptr;
    float* xhatminus_data_ = nullptr;
    float* u_data_ = nullptr;
    float* z_data_ = nullptr;
    float* P_data_ = nullptr;
    float* Pminus_data_ = nullptr;
    float* F_data_ = nullptr;
    float* FT_data_ = nullptr;
    float* B_data_ = nullptr;
    float* H_data_ = nullptr;
    float* HT_data_ = nullptr;
    float* Q_data_ = nullptr;
    float* R_data_ = nullptr;
    float* K_data_ = nullptr;
    float* S_data_ = nullptr;
    float* temp_matrix_data_ = nullptr;
    float* temp_matrix_data1_ = nullptr;
    float* temp_vector_data_ = nullptr;
    float* temp_vector_data1_ = nullptr;
    uint8_t* measurement_map_ = nullptr;
    uint8_t* temp_ = nullptr;
    float* measurement_degree_ = nullptr;
    float* r_diagonal_ = nullptr;
    float* state_min_variance_ = nullptr;

    uint8_t xhat_size_ = 0;
    uint8_t u_size_ = 0;
    uint8_t z_size_ = 0;
    uint8_t measurement_valid_num_ = 0;
    bool use_auto_adjustment_ = false;
    int8_t mat_status_ = 0;

    bool skip_eq1_ = false, skip_eq2_ = false, skip_eq3_ = false;
    bool skip_eq4_ = false, skip_eq5_ = false;

    Hook func0_ = nullptr, func1_ = nullptr, func2_ = nullptr, func3_ = nullptr;
    Hook func4_ = nullptr, func5_ = nullptr, func6_ = nullptr;
    void* hook_ctx_ = nullptr;
};

#endif  // THREE_FRICTION_KALMAN_FILTER_HPP
