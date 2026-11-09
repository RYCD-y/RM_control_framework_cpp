#ifndef THREE_FRICTION_DRV_ZDT_MOTOR_HPP
#define THREE_FRICTION_DRV_ZDT_MOTOR_HPP

#include <cstddef>
#include <cstdint>

#include "hal_uart.hpp"
#include "motor_base.hpp"

struct ZdtMotorConfig {
    // 每圈脉冲数（对应驱动器细分，16 细分 = 3200）
    uint16_t pulses_per_revolution = 3200U;
    // 位置模式默认运行速度（RPM）
    uint16_t default_velocity_rpm = 1000U;
    // 加速度档位（0 = 直接启动）
    uint8_t default_acceleration = 0U;
};

class ZdtMotor final : public motor::IVelocityControl,
                        public motor::IPositionControl,
                        public motor::IEnableControl {
   public:
    // 构造：向总线登记地址并注册接收回调；不碰硬件
    ZdtMotor(UartBus& uart, uint8_t address, const ZdtMotorConfig& config = {});

    // 保存目标速度（RPM）并切到速度模式
    motor::Status SetTargetVelocityRpm(float velocity_rpm) override;
    // 下发速度帧（地址 0xF6）
    motor::Status SendVelocity() override;
    // 保存目标位置（度）并切到位置模式
    motor::Status SetTargetPositionDeg(float position_deg) override;

    // 按当前模式提交（速度 or 位置）
    motor::Status Send() { return velocity_mode_ ? SendVelocity() : SendPosition(); }
    // 下发绝对位置帧（地址 0xFD）
    motor::Status SendPosition() override;

    // 使能/失能（0xF3 AB）；带清除错误选项供具体场景使用
    motor::Status Enable(bool enabled, bool clear_errors = false);
    // 接口实现：使能
    motor::Status Enable() override { return Enable(true, false); }
    // 接口实现：失能
    motor::Status Disable() override { return Enable(false, false); }
    // 立即停止（0xFE 98）
    motor::Status Stop();
    // 当前位置设为零（0x0A 6D）
    motor::Status SetZero();
    // 查询实时位置/转速/状态标志
    motor::Status QueryPosition();
    // 查询实时转速
    motor::Status QueryVelocity();
    // 查询状态标志
    motor::Status QueryStatus();

    // 反馈：位置/速度/状态/错误码/是否有在途应答
    float PositionDeg() const { return position_deg_; }
    float VelocityRpm() const { return velocity_rpm_; }
    uint8_t StatusFlags() const { return status_flags_; }
    uint8_t ErrorCode() const { return error_code_; }
    bool ResponsePending() const { return response_pending_; }
    // 标记期望应答的功能码（同一总线只允许一个在途命令）
    void BeginPending(uint8_t function);
    // 清除在途标记
    void ResetPending();
    // 处理一帧已校验的应答
    void Receive(const uint8_t* data, size_t size);

   private:
    // 总线回调入口
    static void OnReceive(void* context, const uint8_t* data, size_t size);
    // 总线错误回调入口
    static void OnError(void* context, uint32_t error_code);
    // 记录错误并清在途标记
    void Error(uint32_t error_code);
    // 加校验后入队发送，并登记在途应答
    motor::Status SendFrame(const uint8_t* data, size_t size, uint8_t function);
    // 组装速度帧
    motor::Status SendVelocityFrame(float velocity_rpm);
    // 组装绝对位置帧
    motor::Status SendPositionFrame(float position_deg);

    UartBus& uart_;
    uint8_t address_;
    ZdtMotorConfig config_;
    float target_velocity_rpm_ = 0.0f;
    float target_position_deg_ = 0.0f;
    float position_deg_ = 0.0f;
    float velocity_rpm_ = 0.0f;
    uint8_t status_flags_ = 0U;
    uint8_t error_code_ = 0U;
    uint8_t expected_function_ = 0U;
    bool velocity_mode_ = false;
    bool response_pending_ = false;
    uint8_t response_[16] = {};
    size_t response_size_ = 0U;
};

#endif  // THREE_FRICTION_DRV_ZDT_MOTOR_HPP
