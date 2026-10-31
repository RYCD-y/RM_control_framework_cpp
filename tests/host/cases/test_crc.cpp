// Step4 对拍：crc（C++ namespace）vs crc8_crc16（C）
#include <cstdio>
#include <cstdint>

#include "ref_wrap.h"
#include "crc.hpp"

static int g_fail = 0;

static uint32_t rng = 0x12345678u;
static uint8_t  nextByte()
{
        rng = rng * 1664525u + 1013904223u;
        return (uint8_t) (rng >> 16);
}

int main()
{
        uint8_t buf[64];

        for (int t = 0; t < 200; t++)
        {
                const uint32_t len = 1u + (rng % 60u);
                for (uint32_t i = 0; i < len; i++) { buf[i] = nextByte(); }

                const uint8_t  r8  = ref_crc8(buf, len, 0xFF);
                const uint8_t  n8  = crc::crc8(buf, len);
                const uint16_t r16 = ref_crc16(buf, len, 0xFFFF);
                const uint16_t n16 = crc::crc16(buf, len);

                if (r8 != n8) { printf("FAIL crc8[%d] ref=0x%02X new=0x%02X\n", t, r8, n8); g_fail++; }
                if (r16 != n16) { printf("FAIL crc16[%d] ref=0x%04X new=0x%04X\n", t, r16, n16); g_fail++; }

                // append + verify 一致性
                uint8_t a8[64];
                for (uint32_t i = 0; i < len; i++) { a8[i] = buf[i]; }
                crc::append8(a8, len + 1);
                if (!crc::verify8(a8, len + 1)) { printf("FAIL verify8[%d]\n", t); g_fail++; }
                if (a8[len] != r8) { printf("FAIL append8[%d] ref=0x%02X new=0x%02X\n", t, r8, a8[len]); g_fail++; }

                uint8_t a16[64];
                for (uint32_t i = 0; i < len; i++) { a16[i] = buf[i]; }
                crc::append16(a16, len + 2);
                if (!crc::verify16(a16, len + 2)) { printf("FAIL verify16[%d]\n", t); g_fail++; }
                const uint16_t appended = (uint16_t) (a16[len] | ((uint16_t) a16[len + 1] << 8));
                if (appended != r16) { printf("FAIL append16[%d] ref=0x%04X new=0x%04X\n", t, r16, appended); g_fail++; }

                // CRC-CCITT
                const uint16_t rcc = ref_ccitt(0xFFFF, buf, len);
                const uint16_t ncc = crc::ccitt(0xFFFF, buf, len);
                if (rcc != ncc) { printf("FAIL ccitt[%d] ref=0x%04X new=0x%04X\n", t, rcc, ncc); g_fail++; }
        }

        printf("last: crc8=0x%02X crc16=0x%04X ccitt=0x%04X\n",
               crc::crc8(buf, 16), crc::crc16(buf, 16), crc::ccitt(0xFFFF, buf, 16));
        printf(g_fail == 0 ? "test_crc: PASS\n" : "test_crc: %d FAILED\n", g_fail);
        return g_fail == 0 ? 0 : 1;
}
