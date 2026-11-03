#ifndef THREE_FRICTION_MOTOR_BASE_HPP
#define THREE_FRICTION_MOTOR_BASE_HPP

#include <cstdint>

// 控制能力接口：彼此独立，互不继承。
// 上层只依赖自己真正需要的能力，驱动只承诺自己能做的事。
namespace motor {

enum class Status : uint8_t {
    kOk,
    kBusy,
    kUnsupported,
    kInvalidArgument,
    kNotReady,
    kIoError,
};

class ITorqueControl {
   public:
    virtual ~ITorqueControl() = default;
    virtual Status SetTargetTorqueNm(float torque_nm) = 0;
    virtual Status SendTorque() = 0;
};

class IVelocityControl {
   public:
    virtual ~IVelocityControl() = default;
    virtual Status SetTargetVelocityRadPerSec(float velocity_rad_per_sec) = 0;
    virtual Status SendVelocity() = 0;
};

class IPositionControl {
   public:
    virtual ~IPositionControl() = default;
    virtual Status SetTargetPositionRad(float position_rad) = 0;
    virtual Status SendPosition() = 0;
};

}  // namespace motor

#endif  // THREE_FRICTION_MOTOR_BASE_HPP
