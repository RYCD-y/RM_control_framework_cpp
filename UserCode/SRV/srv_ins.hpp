//
// INS（姿态解算）服务
// 仿照 control_framework_v2/UserCode/SRV/srv_ins.c 的 INS_Task，改为 C++ 任务类。
//
#ifndef THREE_FRICTION_SRV_INS_HPP
#define THREE_FRICTION_SRV_INS_HPP

#include <stdint.h>

#include "FreeRTOS.h"
#include "quaternion_ekf.hpp"
#include "task.h"

// 坐标系索引
enum InsAxis : uint8_t {
    InsX = 0,
    InsY = 1,
    InsZ = 2,
};

class Ins {
   public:
    // ── 任务参数 ──
    static constexpr uint32_t kTaskPeriodMs = 1;  // INS_TASK_PERIOD
    static constexpr uint16_t kTaskStack = 512;   // words
    static constexpr UBaseType_t kTaskPriority = 5;
    static constexpr float kRadToDeg = 57.295779513f;

    void Init();

    // ── FreeRTOS 任务接入 ──
    static void TaskEntry(void* argument);  // 传 this 进来
    void Run();                             // 任务主体（vTaskDelayUntil 循环）
    bool Start(const char* name = "INS");

    // ── 数据更新 ──
    // 取一次 IMU 数据并更新姿态（v2 的 INS_Task 主体）
    // TODO: IMU 驱动接入前，Body 数据由外部灌入；Update() 留空
    void Update();
    void Step(float Dt, const float Gyro[3], const float Accel[3]);
    void Task(void* argument);

    // ── 安装误差修正 ──
    void SetMountParam(float YawDeg, float PitchDeg, float RollDeg);
    void SetMountScale(float sx, float sy, float sz);

    // ── 只读查询 ──
    const float* Quaternion() const { return ekf_.Quaternion(); }
    const float* Gyro() const { return gyro_; }
    const float* Accel() const { return accel_; }
    const float* MotionAccelBody() const { return motion_accel_b_; }
    const float* MotionAccelEarth() const { return motion_accel_n_; }
    const float* AxisX() const { return xn_; }
    const float* AxisY() const { return yn_; }
    const float* AxisZ() const { return zn_; }

    float RollDeg() const { return ekf_.RollDeg(); }    // 度
    float PitchDeg() const { return ekf_.PitchDeg(); }  // 度
    float YawDeg() const { return ekf_.YawDeg(); }      // 度
    float YawTotalAngleDeg() const { return ekf_.YawTotalAngleDeg(); }
    float AtanXZDeg() const { return atan_xz_; }  // 度
    float AtanYZDeg() const { return atan_yz_; }  // 度

    bool Ready() const { return initialized_; }
    bool Converged() const { return ekf_.Converged(); }
    uint32_t SampleCount() const { return sample_count_; }
    float Dt() const { return dt_; }

    // ── 姿态数学工具（无状态）──
    static void QuaternionUpdate(float q[4], float gx, float gy, float gz, float Dt);
    static void QuaternionToEuler(const float q[4], float& YawDeg, float& PitchDeg, float& RollDeg);
    static void EulerToQuaternion(float YawDeg, float PitchDeg, float RollDeg, float q[4]);
    static void BodyToEarth(const float vec_b[3], float vec_e[3], const float q[4]);
    static void EarthToBody(const float vec_e[3], float vec_b[3], const float q[4]);

    // 初始化安装误差参数用于修正
    struct MountParam {
        bool dirty = true;
        float scale[3] = {1.0f, 1.0f, 1.0f};
        float YawDeg = 0.0f, PitchDeg = 0.0f, RollDeg = 0.0f;
    };

   private:
    void ApplyMountCorrection(float Gyro[3], float Accel[3]);  // v2 的 IMU_Param_Correction
    void BuildRotationMatrix();                                // 由 mount_ 生成 c_[3][3]

    QuaternionEkf ekf_;

    float gyro_[3] = {};
    float accel_[3] = {};
    float motion_accel_b_[3] = {};      // 机体系运动加速度
    float motion_accel_n_[3] = {};      // 导航系运动加速度
    float xn_[3] = {1.0f, 0.0f, 0.0f};  // 机体系基向量在导航系下的表示
    float yn_[3] = {0.0f, 1.0f, 0.0f};
    float zn_[3] = {0.0f, 0.0f, 1.0f};
    float atan_xz_ = 0.0f;
    float atan_yz_ = 0.0f;
    float accel_lpf_ = 0.0085f;  // 运动加速度低通系数
    float dt_ = 0.0f;
    float time_ = 0.0f;

    uint32_t dwt_cnt_ = 0;
    uint32_t count_ = 0;
    uint32_t sample_count_ = 0;
    bool initialized_ = false;

    MountParam mount_;
    float c_[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    float last_yaw_offset_ = 0.0f;
    float last_pitch_offset_ = 0.0f;
    float last_roll_offset_ = 0.0f;
};

/// 全局实例（与 v2 的 INS 对应）
extern Ins g_ins;

#endif  // THREE_FRICTION_SRV_INS_HPP
