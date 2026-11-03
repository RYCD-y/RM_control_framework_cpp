//
// RoboMaster 裁判系统 / 通用 CRC8、CRC16、CRC-CCITT
// 对应 control_framework_v2/UserCode/Common/crc8_crc16.c，改为无状态 namespace
//
#ifndef THREE_FRICTION_CRC_HPP
#define THREE_FRICTION_CRC_HPP

#include <stddef.h>
#include <stdint.h>

namespace crc {
inline constexpr uint8_t kInit8 = 0xFFU;
inline constexpr uint16_t kInit16 = 0xFFFFU;

// ── CRC8（G(X)=X^8+X^5+X^4+1），Init 默认 0xFF ──
uint8_t Crc8(const uint8_t* message, uint32_t length, uint8_t initial = kInit8);
bool Verify8(const uint8_t* message, uint32_t length);
void Append8(uint8_t* message, uint32_t length);

uint16_t Crc16(const uint8_t* message, uint32_t length, uint16_t initial = kInit16);
bool Verify16(const uint8_t* message, uint32_t length);
void Append16(uint8_t* message, uint32_t length);

uint16_t CcittByte(uint16_t value, uint8_t byte);
uint16_t Ccitt(uint16_t value, const uint8_t* buffer, size_t length);
}  // namespace crc

#endif  // THREE_FRICTION_CRC_HPP
