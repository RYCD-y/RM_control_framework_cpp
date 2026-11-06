#include "drv_zdt_motor.hpp"

#include <algorithm>
#include <cmath>

namespace {
// 默认固定校验字节（Emm V5 出厂设置）
constexpr uint8_t kChecksum = 0x6BU;
constexpr float kDegPerRevolution = 360.0f;
// 命令应答超时：超时则允许下一条命令
constexpr TickType_t kResponseTimeoutTicks = pdMS_TO_TICKS(30U);
constexpr size_t kRxAccumulatorSize = 64U;
// 每条 UART 总线一份：地址表 + 字节累积区 + 在途命令
struct BusSlot {
    UartBus* uart = nullptr;
    std::array<ZdtMotor*, 16> motors{};
    std::array<uint8_t, kRxAccumulatorSize> bytes{};
    size_t size = 0U;
    ZdtMotor* pending = nullptr;
    TickType_t pending_since = 0U;
};
std::array<BusSlot, 3> g_buses{};

// 按总线查槽位，新总线则分配一个空槽
BusSlot* FindBus(UartBus& uart) {
    for (BusSlot& bus : g_buses)
        if (bus.uart == &uart) return &bus;
    for (BusSlot& bus : g_buses) {
        if (bus.uart == nullptr) {
            bus.uart = &uart;
            return &bus;
        }
    }
    return nullptr;
}

// 累加和校验（不含末字节本身）
uint8_t Checksum(const uint8_t* data, size_t size) {
    uint8_t sum = 0U;
    for (size_t i = 0U; i + 1U < size; ++i) sum = static_cast<uint8_t>(sum + data[i]);
    return sum;
}

// 按功能码查帧长：0 表示未知功能码
size_t FrameLength(uint8_t function) {
    switch (function) {
        case 0x00U:
        case 0x3AU:
        case 0xF3U:
        case 0xF6U:
        case 0xFDU:
        case 0xFEU:
        case 0xFFU:
            return 4U;
        case 0x35U:
            return 6U;
        case 0x32U:
        case 0x33U:
        case 0x36U:
            return 8U;
        default:
            return 0U;
    }
}

// 从累积字节中切出完整帧并分发给对应地址的电机
void Dispatch(BusSlot& bus) {
    size_t offset = 0U;
    while (bus.size - offset >= 2U) {
        const uint8_t address = bus.bytes[offset];
        if (address == 0U || address >= bus.motors.size() || bus.motors[address] == nullptr) {
            ++offset;
            continue;
        }
        const size_t length = FrameLength(bus.bytes[offset + 1U]);
        if (length == 0U) {
            ++offset;
            continue;
        }
        if (bus.size - offset < length) break;
        if (bus.bytes[offset + length - 1U] == kChecksum &&
            Checksum(bus.bytes.data() + offset, length) == kChecksum) {
            bus.motors[address]->Receive(bus.bytes.data() + offset, length);
            offset += length;
        } else {
            ++offset;
        }
    }
    if (offset != 0U) {
        bus.size -= offset;
        std::move(bus.bytes.begin() + offset, bus.bytes.begin() + offset + bus.size,
                  bus.bytes.begin());
    }
}

// 总线收到字节：追加到累积区后尝试分帧
void OnBusReceive(void* context, const uint8_t* data, size_t size) {
    auto& bus = *static_cast<BusSlot*>(context);
    if (size > bus.bytes.size() - bus.size) {
        bus.size = 0U;
        bus.pending = nullptr;
    }
    const size_t copied = std::min(size, bus.bytes.size() - bus.size);
    std::copy(data, data + copied, bus.bytes.begin() + bus.size);
    bus.size += copied;
    Dispatch(bus);
}

// 总线错误：丢弃半帧并清在途命令
void OnBusError(void* context, uint32_t) {
    auto& bus = *static_cast<BusSlot*>(context);
    bus.size = 0U;
    bus.pending = nullptr;
}

}  // namespace

// 登记期望应答的功能码，进入在途状态
void ZdtMotor::BeginPending(uint8_t function) {
    expected_function_ = function;
    response_pending_ = true;
    response_size_ = 0U;
}

// 退出在途状态
void ZdtMotor::ResetPending() { response_pending_ = false; }

// 构造：校验地址/配置并向总线登记本机；失败时标记 NotReady
ZdtMotor::ZdtMotor(UartBus& uart, uint8_t address, const ZdtMotorConfig& config)
    : uart_(uart), address_(address), config_(config) {
    BusSlot* bus = FindBus(uart_);
    if (address_ == 0U || address_ >= 16U || bus == nullptr || bus->motors[address_] != nullptr ||
        !uart_.IsStarted() || config_.pulses_per_revolution == 0U ||
        uart_.RegisterCallback(OnBusReceive, OnBusError, bus) != HAL_OK) {
        error_code_ = 0xFFU;
        return;
    }
    bus->motors[address_] = this;
}

// 总线接收回调入口
void ZdtMotor::OnReceive(void* context, const uint8_t* data, size_t size) {
    static_cast<ZdtMotor*>(context)->Receive(data, size);
}

// 总线错误回调入口
void ZdtMotor::OnError(void* context, uint32_t error_code) {
    static_cast<ZdtMotor*>(context)->Error(error_code);
}

// 校验后按功能码解析：状态/速度/位置；在途命令由应答解除
void ZdtMotor::Receive(const uint8_t* data, size_t size) {
    if (data == nullptr || size < 4U || data[0] != address_ || Checksum(data, size) != kChecksum ||
        size != FrameLength(data[1]))
        return;
    if (data[1] == 0x00U) {
        error_code_ = data[2];
        response_pending_ = false;
        return;
    }
    if (response_pending_ && data[1] != expected_function_) return;
    switch (data[1]) {
        case 0x3AU:
            status_flags_ = data[2];
            break;
        case 0x35U: {
            const int16_t rpm = static_cast<int16_t>((data[3] << 8U) | data[4]);
            const float direction = data[2] == 0U ? 1.0f : -1.0f;
            velocity_deg_per_sec_ = direction * static_cast<float>(rpm) * kDegPerRevolution / 60.0f;
            break;
        }
        case 0x36U: {
            int32_t raw = static_cast<int32_t>((static_cast<uint32_t>(data[3]) << 24U) |
                                               (static_cast<uint32_t>(data[4]) << 16U) |
                                               (static_cast<uint32_t>(data[5]) << 8U) | data[6]);
            if (data[2] != 0U) raw = -raw;
            position_deg_ = static_cast<float>(raw) * kDegPerRevolution / 65536.0f;
            break;
        }
        default:
            break;
    }
    if (response_pending_) {
        response_pending_ = false;
        const size_t copy_size = size < sizeof(response_) ? size : sizeof(response_);
        response_size_ = copy_size;
        for (size_t i = 0; i < copy_size; ++i) response_[i] = data[i];
        BusSlot* bus = FindBus(uart_);
        if (bus != nullptr && bus->pending == this) bus->pending = nullptr;
    }
}

// 记录驱动错误并清除在途标记
void ZdtMotor::Error(uint32_t error_code) {
    error_code_ = static_cast<uint8_t>(error_code);
    response_pending_ = false;
}

// 加校验后入队发送；同总线已有在途命令时返回 Busy
motor::Status ZdtMotor::SendFrame(const uint8_t* data, size_t size, uint8_t function) {
    BusSlot* bus = FindBus(uart_);
    if (bus == nullptr || error_code_ == 0xFFU || data == nullptr || size < 4U)
        return motor::Status::kNotReady;
    if (bus->pending != nullptr && bus->pending != this &&
        xTaskGetTickCount() - bus->pending_since < kResponseTimeoutTicks)
        return motor::Status::kBusy;
    if (bus->pending == this) bus->pending = nullptr;
    BeginPending(function);
    bus->pending = this;
    bus->pending_since = xTaskGetTickCount();
    const HAL_StatusTypeDef status = uart_.Send(data, size);
    if (status != HAL_OK) {
        bus->pending = nullptr;
        response_pending_ = false;
        return status == HAL_BUSY ? motor::Status::kBusy : motor::Status::kIoError;
    }
    return motor::Status::kOk;
}

// 保存目标速度并切到速度模式
motor::Status ZdtMotor::SetTargetVelocityDegPerSec(float velocity_deg_per_sec) {
    if (!std::isfinite(velocity_deg_per_sec)) return motor::Status::kInvalidArgument;
    target_velocity_deg_per_sec_ = velocity_deg_per_sec;
    velocity_mode_ = true;
    return motor::Status::kOk;
}

// 保存目标位置并切到位置模式
motor::Status ZdtMotor::SetTargetPositionDeg(float position_deg) {
    if (!std::isfinite(position_deg)) return motor::Status::kInvalidArgument;
    target_position_deg_ = position_deg;
    velocity_mode_ = false;
    return motor::Status::kOk;
}

// 提交速度目标
motor::Status ZdtMotor::SendVelocity() { return SendVelocityFrame(target_velocity_deg_per_sec_); }

// 提交位置目标
motor::Status ZdtMotor::SendPosition() { return SendPositionFrame(target_position_deg_); }

// 组帧：地址+0xF6+方向+转速(16位)+加速度+同步+校验
motor::Status ZdtMotor::SendVelocityFrame(float velocity_deg_per_sec) {
    const float rpm = std::fabs(velocity_deg_per_sec) * 60.0f / kDegPerRevolution;
    if (rpm > 5000.0f) return motor::Status::kInvalidArgument;
    const uint16_t raw = static_cast<uint16_t>(rpm);
    const uint8_t frame[8] = {address_,
                              0xF6U,
                              static_cast<uint8_t>(velocity_deg_per_sec < 0.0f ? 1U : 0U),
                              static_cast<uint8_t>(raw >> 8U),
                              static_cast<uint8_t>(raw),
                              config_.default_acceleration,
                              0U,
                              kChecksum};
    return SendFrame(frame, sizeof(frame), frame[1]);
}

// 组帧：地址+0xFD+方向+速度+加速度+脉冲数+绝对模式+同步+校验
motor::Status ZdtMotor::SendPositionFrame(float position_deg) {
    if (config_.pulses_per_revolution == 0U) return motor::Status::kInvalidArgument;
    const double pulses =
        std::fabs(static_cast<double>(position_deg) / static_cast<double>(kDegPerRevolution) *
                  config_.pulses_per_revolution);
    if (pulses > 4294967295.0) return motor::Status::kInvalidArgument;
    const uint32_t raw = static_cast<uint32_t>(pulses);
    const uint16_t rpm = config_.default_velocity_rpm;
    const uint8_t frame[13] = {address_,
                               0xFDU,
                               static_cast<uint8_t>(position_deg < 0.0f ? 1U : 0U),
                               static_cast<uint8_t>(rpm >> 8U),
                               static_cast<uint8_t>(rpm),
                               config_.default_acceleration,
                               static_cast<uint8_t>(raw >> 24U),
                               static_cast<uint8_t>(raw >> 16U),
                               static_cast<uint8_t>(raw >> 8U),
                               static_cast<uint8_t>(raw),
                               1U,
                               0U,
                               kChecksum};
    return SendFrame(frame, sizeof(frame), frame[1]);
}

// 使能/失能（0xF3 AB，末字节为多机同步标志）
motor::Status ZdtMotor::Enable(bool enabled, bool clear_errors) {
    const uint8_t frame[6] = {
        address_, 0xF3U, 0xABU, static_cast<uint8_t>(enabled), static_cast<uint8_t>(clear_errors),
        kChecksum};
    return SendFrame(frame, sizeof(frame), frame[1]);
}

// 立即停止
motor::Status ZdtMotor::Stop() {
    const uint8_t frame[5] = {address_, 0xFEU, 0x98U, 0U, kChecksum};
    return SendFrame(frame, sizeof(frame), frame[1]);
}

// 当前位置清零
motor::Status ZdtMotor::SetZero() {
    const uint8_t frame[4] = {address_, 0x0AU, 0x6DU, kChecksum};
    return SendFrame(frame, sizeof(frame), frame[1]);
}

// 读实时位置（0x36）
motor::Status ZdtMotor::QueryPosition() {
    const uint8_t frame[3] = {address_, 0x36U, kChecksum};
    return SendFrame(frame, sizeof(frame), frame[1]);
}

// 读实时转速（0x35）
motor::Status ZdtMotor::QueryVelocity() {
    const uint8_t frame[3] = {address_, 0x35U, kChecksum};
    return SendFrame(frame, sizeof(frame), frame[1]);
}

// 读状态标志（0x3A）
motor::Status ZdtMotor::QueryStatus() {
    const uint8_t frame[3] = {address_, 0x3AU, kChecksum};
    return SendFrame(frame, sizeof(frame), frame[1]);
}
