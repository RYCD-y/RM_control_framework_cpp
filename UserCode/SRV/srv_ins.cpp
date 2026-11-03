//
// INS 服务实现（对应 v2 的 INS_Task / INS_Init / IMU_Param_Correction）
//
#include "srv_ins.hpp"

#include <cmath>
#include <cstring>

#include "arm_math.h"
#include "dwt.hpp"

Ins g_ins;

/* ═══════════════════════ 初始化 ═══════════════════════ */
void Ins::Init() {
    mount_ = MountParam{};  // scale=1、角度=0、dirty=true
    BuildRotationMatrix();

    // v2: IMU_QuaternionEKF_Init(10.0f, 0.001f, 10000000, 1, 0)
    ekf_.Init(10.0f, 0.001f, 10000000.0f, 1.0f, 0.0f);

    accel_lpf_ = 0.0085f;

    dt_ = 0.0f;
    time_ = 0.0f;
    count_ = 0;
    sample_count_ = 0;
    initialized_ = true;
}

/* ═══════════════════════ 任务接入 ═══════════════════════ */
void Ins::TaskEntry(void* argument) { static_cast<Ins*>(argument)->Run(); }

void Ins::Run() {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // v2: Dt 在任务里统一取，再交给 Update()
        dt_ = Dwt::DeltaT(dwt_cnt_);

        Update();

        count_++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kTaskPeriodMs));
    }
}

bool Ins::Start(const char* name) {
    return xTaskCreate(&Ins::TaskEntry, name, kTaskStack, this, kTaskPriority, nullptr) == pdPASS;
}

/* ═══════════════════════ 数据更新 ═══════════════════════ */
void Ins::Update() {
    // TODO: IMU 驱动接入后在此取数并调 Step()：
    //   IMU_Read_Data(&himu1);
    //   Step(dt_, IMU_Get_Gyro(&himu1), IMU_Get_Accel(&himu1));
    //
    // 当前 IMU 接口留空：可先由外部调用 Step(Dt, Gyro, Accel) 灌数据。
}

void Ins::Step(float Dt, const float Gyro[3], const float Accel[3]) {
    if (!initialized_) {
        Init();
    }

    dt_ = Dt;
    time_ += dt_;

    gyro_[InsX] = Gyro[InsX];
    gyro_[InsY] = Gyro[InsY];
    gyro_[InsZ] = Gyro[InsZ];
    accel_[InsX] = Accel[InsX];
    accel_[InsY] = Accel[InsY];
    accel_[InsZ] = Accel[InsZ];

    // 安装误差修正（demo 用，默认单位阵）
    ApplyMountCorrection(gyro_, accel_);

    // 重力加速度矢量与机体系 XZ / YZ 平面的夹角（功能扩展用）
    atan_xz_ = -atan2f(accel_[InsX], accel_[InsZ]) * 180.0f / 3.14159265354f;
    atan_yz_ = atan2f(accel_[InsY], accel_[InsZ]) * 180.0f / 3.14159265354f;

    // 核心：EKF 更新四元数
    ekf_.Update(gyro_[InsX], gyro_[InsY], gyro_[InsZ], accel_[InsX], accel_[InsY], accel_[InsZ],
                dt_);

    const float* q = ekf_.Quaternion();

    // 机体系基向量转到导航系（输入必须是固定的 xb/yb/zb，不能拿上一帧结果再变换）
    static constexpr float kAxisBody[3][3] = {
        {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    BodyToEarth(kAxisBody[0], xn_, q);
    BodyToEarth(kAxisBody[1], yn_, q);
    BodyToEarth(kAxisBody[2], zn_, q);

    // 重力从导航系转到机体系，再算运动加速度
    const float gravity[3] = {0.0f, 0.0f, 9.81f};
    float gravity_b[3] = {};
    EarthToBody(gravity, gravity_b, q);

    for (uint8_t i = 0; i < 3; i++) {
        motion_accel_b_[i] = (accel_[i] - gravity_b[i]) * dt_ / (accel_lpf_ + dt_) +
                             motion_accel_b_[i] * accel_lpf_ / (accel_lpf_ + dt_);
    }
    BodyToEarth(motion_accel_b_, motion_accel_n_, q);

    sample_count_++;
}

/* ═══════════════════════ 安装误差修正 ═══════════════════════ */
void Ins::SetMountParam(float YawDeg, float PitchDeg, float RollDeg) {
    mount_.YawDeg = YawDeg;
    mount_.PitchDeg = PitchDeg;
    mount_.RollDeg = RollDeg;
    mount_.dirty = true;
}

void Ins::SetMountScale(float sx, float sy, float sz) {
    mount_.scale[InsX] = sx;
    mount_.scale[InsY] = sy;
    mount_.scale[InsZ] = sz;
}

void Ins::BuildRotationMatrix() {
    const float cos_yaw = arm_cos_f32(mount_.YawDeg / kRadToDeg);
    const float cos_pitch = arm_cos_f32(mount_.PitchDeg / kRadToDeg);
    const float cos_roll = arm_cos_f32(mount_.RollDeg / kRadToDeg);
    const float sin_yaw = arm_sin_f32(mount_.YawDeg / kRadToDeg);
    const float sin_pitch = arm_sin_f32(mount_.PitchDeg / kRadToDeg);
    const float sin_roll = arm_sin_f32(mount_.RollDeg / kRadToDeg);

    // 1.YawDeg(alpha) 2.PitchDeg(beta) 3.RollDeg(gamma)
    c_[0][0] = cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll;
    c_[0][1] = cos_pitch * sin_yaw;
    c_[0][2] = cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch;
    c_[1][0] = cos_yaw * sin_pitch * sin_roll - cos_roll * sin_yaw;
    c_[1][1] = cos_yaw * cos_pitch;
    c_[1][2] = -sin_yaw * sin_roll - cos_yaw * cos_roll * sin_pitch;
    c_[2][0] = -cos_pitch * sin_roll;
    c_[2][1] = sin_pitch;
    c_[2][2] = cos_pitch * cos_roll;

    mount_.dirty = false;
}

void Ins::ApplyMountCorrection(float Gyro[3], float Accel[3]) {
    if (mount_.dirty || fabsf(mount_.YawDeg - last_yaw_offset_) > 0.001f ||
        fabsf(mount_.PitchDeg - last_pitch_offset_) > 0.001f ||
        fabsf(mount_.RollDeg - last_roll_offset_) > 0.001f) {
        BuildRotationMatrix();
        last_yaw_offset_ = mount_.YawDeg;
        last_pitch_offset_ = mount_.PitchDeg;
        last_roll_offset_ = mount_.RollDeg;
    }

    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++) {
        gyro_temp[i] = Gyro[i] * mount_.scale[i];
    }

    for (uint8_t row = 0; row < 3; row++) {
        Gyro[row] =
            c_[row][0] * gyro_temp[0] + c_[row][1] * gyro_temp[1] + c_[row][2] * gyro_temp[2];
    }

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++) {
        accel_temp[i] = Accel[i];
    }

    for (uint8_t row = 0; row < 3; row++) {
        Accel[row] =
            c_[row][0] * accel_temp[0] + c_[row][1] * accel_temp[1] + c_[row][2] * accel_temp[2];
    }
}

/* ═══════════════════════ 姿态数学工具 ═══════════════════════ */
void Ins::QuaternionUpdate(float q[4], float gx, float gy, float gz, float Dt) {
    gx *= 0.5f * Dt;
    gy *= 0.5f * Dt;
    gz *= 0.5f * Dt;

    const float qa = q[0];
    const float qb = q[1];
    const float qc = q[2];

    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

void Ins::QuaternionToEuler(const float q[4], float& YawDeg, float& PitchDeg, float& RollDeg) {
    YawDeg = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) *
             kRadToDeg;
    PitchDeg =
        atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) *
        kRadToDeg;
    RollDeg = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * kRadToDeg;
}

void Ins::EulerToQuaternion(float YawDeg, float PitchDeg, float RollDeg, float q[4]) {
    YawDeg /= kRadToDeg;
    PitchDeg /= kRadToDeg;
    RollDeg /= kRadToDeg;

    const float cos_pitch = arm_cos_f32(PitchDeg / 2.0f);
    const float cos_yaw = arm_cos_f32(YawDeg / 2.0f);
    const float cos_roll = arm_cos_f32(RollDeg / 2.0f);
    const float sin_pitch = arm_sin_f32(PitchDeg / 2.0f);
    const float sin_yaw = arm_sin_f32(YawDeg / 2.0f);
    const float sin_roll = arm_sin_f32(RollDeg / 2.0f);

    q[0] = cos_pitch * cos_roll * cos_yaw + sin_pitch * sin_roll * sin_yaw;
    q[1] = sin_pitch * cos_roll * cos_yaw - cos_pitch * sin_roll * sin_yaw;
    q[2] = sin_pitch * cos_roll * sin_yaw + cos_pitch * sin_roll * cos_yaw;
    q[3] = cos_pitch * cos_roll * sin_yaw - sin_pitch * sin_roll * cos_yaw;
}

void Ins::BodyToEarth(const float vec_b[3], float vec_e[3], const float q[4]) {
    const float X = vec_b[0];
    const float y = vec_b[1];
    const float z = vec_b[2];

    vec_e[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * X + (q[1] * q[2] - q[0] * q[3]) * y +
                       (q[1] * q[3] + q[0] * q[2]) * z);

    vec_e[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * X + (0.5f - q[1] * q[1] - q[3] * q[3]) * y +
                       (q[2] * q[3] - q[0] * q[1]) * z);

    vec_e[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * X + (q[2] * q[3] + q[0] * q[1]) * y +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * z);
}

void Ins::EarthToBody(const float vec_e[3], float vec_b[3], const float q[4]) {
    const float X = vec_e[0];
    const float y = vec_e[1];
    const float z = vec_e[2];

    vec_b[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * X + (q[1] * q[2] + q[0] * q[3]) * y +
                       (q[1] * q[3] - q[0] * q[2]) * z);

    vec_b[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * X + (0.5f - q[1] * q[1] - q[3] * q[3]) * y +
                       (q[2] * q[3] + q[0] * q[1]) * z);

    vec_b[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * X + (q[2] * q[3] - q[0] * q[1]) * y +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * z);
}
