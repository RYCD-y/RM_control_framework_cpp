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
enum InsAxis : uint8_t
{
        InsX = 0,
        InsY = 1,
        InsZ = 2,
};

class Ins
{
        public:
                // ── 任务参数 ──
                static constexpr uint32_t    kTaskPeriodMs = 1;      // INS_TASK_PERIOD
                static constexpr uint16_t    kTaskStack    = 512;    // words
                static constexpr UBaseType_t kTaskPriority = 5;
                static constexpr float       kRadToDeg     = 57.295779513f;

                void init();

                // ── FreeRTOS 任务接入 ──
                static void taskEntry(void *argument);   // 传 this 进来
                void        run();                       // 任务主体（vTaskDelayUntil 循环）
                bool        start(const char *name = "INS");

                // ── 数据更新 ──
                // 取一次 IMU 数据并更新姿态（v2 的 INS_Task 主体）
                // TODO: IMU 驱动接入前，Body 数据由外部灌入；update() 留空
                void update();
                void step(float dt, const float gyro[3], const float accel[3]);
                void task(void *argument);

                // ── 安装误差修正 ──
                void setMountParam(float yaw, float pitch, float roll);
                void setMountScale(float sx, float sy, float sz);

                // ── 只读查询 ──
                const float *quaternion() const { return ekf_.quaternion(); }
                const float *gyro() const { return gyro_; }
                const float *accel() const { return accel_; }
                const float *motionAccelBody() const { return motion_accel_b_; }
                const float *motionAccelEarth() const { return motion_accel_n_; }
                const float *axisX() const { return xn_; }
                const float *axisY() const { return yn_; }
                const float *axisZ() const { return zn_; }

                float roll() const { return ekf_.roll(); }        // 度
                float pitch() const { return ekf_.pitch(); }      // 度
                float yaw() const { return ekf_.yaw(); }          // 度
                float yawTotalAngle() const { return ekf_.yawTotalAngle(); }
                float atanXZ() const { return atan_xz_; }         // 度
                float atanYZ() const { return atan_yz_; }         // 度

                bool     ready() const { return initialized_; }
                bool     converged() const { return ekf_.converged(); }
                uint32_t sampleCount() const { return sample_count_; }
                float    dt() const { return dt_; }

                // ── 姿态数学工具（无状态）──
                static void quaternionUpdate(float q[4], float gx, float gy, float gz, float dt);
                static void quaternionToEuler(const float q[4], float &yaw, float &pitch, float &roll);
                static void eulerToQuaternion(float yaw, float pitch, float roll, float q[4]);
                static void bodyToEarth(const float vec_b[3], float vec_e[3], const float q[4]);
                static void earthToBody(const float vec_e[3], float vec_b[3], const float q[4]);

                // 初始化安装误差参数用于修正
                struct MountParam
                {
                        bool  dirty = true;
                        float scale[3] = { 1.0f, 1.0f, 1.0f };
                        float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
                };

        private:
                void applyMountCorrection(float gyro[3], float accel[3]);   // v2 的 IMU_Param_Correction
                void buildRotationMatrix();                                 // 由 mount_ 生成 c_[3][3]

                QuaternionEkf ekf_;

                float    gyro_[3]  = {};
                float    accel_[3] = {};
                float    motion_accel_b_[3] = {};   // 机体系运动加速度
                float    motion_accel_n_[3] = {};   // 导航系运动加速度
                float    xn_[3] = { 1.0f, 0.0f, 0.0f };   // 机体系基向量在导航系下的表示
                float    yn_[3] = { 0.0f, 1.0f, 0.0f };
                float    zn_[3] = { 0.0f, 0.0f, 1.0f };
                float    atan_xz_ = 0.0f;
                float    atan_yz_ = 0.0f;
                float    accel_lpf_ = 0.0085f;      // 运动加速度低通系数
                float    dt_ = 0.0f;
                float    time_ = 0.0f;

                uint32_t dwt_cnt_ = 0;
                uint32_t count_ = 0;
                uint32_t sample_count_ = 0;
                bool     initialized_ = false;

                MountParam mount_;
                float      c_[3][3] = { { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };
                float      last_yaw_offset_ = 0.0f;
                float      last_pitch_offset_ = 0.0f;
                float      last_roll_offset_ = 0.0f;
};

/// 全局实例（与 v2 的 INS 对应）
extern Ins g_ins;

#endif //THREE_FRICTION_SRV_INS_HPP
