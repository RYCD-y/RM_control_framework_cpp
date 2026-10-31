//
// RoboMaster 裁判系统 / 通用 CRC8、CRC16、CRC-CCITT
// 对应 control_framework_v2/UserCode/Common/crc8_crc16.c，改为无状态 namespace
//
#ifndef THREE_FRICTION_CRC_HPP
#define THREE_FRICTION_CRC_HPP

#include <stdint.h>
#include <stddef.h>

namespace crc
{
        inline constexpr uint8_t  kInit8  = 0xFFU;
        inline constexpr uint16_t kInit16 = 0xFFFFU;

        // ── CRC8（G(x)=x^8+x^5+x^4+1），init 默认 0xFF ──
        uint8_t  crc8(const uint8_t *msg, uint32_t len, uint8_t init = kInit8);
        bool     verify8(const uint8_t *msg, uint32_t len);       // len = 数据 + 1 字节校验
        void     append8(uint8_t *msg, uint32_t len);             // 校验写入末尾 1 字节

        // ── CRC16（RoboMaster 表驱动），init 默认 0xFFFF ──
        uint16_t crc16(const uint8_t *msg, uint32_t len, uint16_t init = kInit16);
        bool     verify16(const uint8_t *msg, uint32_t len);      // len = 数据 + 2 字节校验
        void     append16(uint8_t *msg, uint32_t len);            // 校验写入末尾 2 字节（小端）

        // ── CRC-CCITT ──
        uint16_t ccittByte(uint16_t value, uint8_t c);
        uint16_t ccitt(uint16_t value, const uint8_t *buf, size_t len);
} // namespace crc

#endif //THREE_FRICTION_CRC_HPP
