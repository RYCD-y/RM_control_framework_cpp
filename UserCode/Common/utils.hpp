//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_UTILS_HPP
#define THREE_FRICTION_UTILS_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace utils
{
        // 物理量 → 协议 raw。x_max 对应 IntType 的最大值（负侧取 min）。
        // 模板必须定义在头文件里，否则别的 .cpp 无法实例化。
        template<typename IntType>
        IntType float_to_signed(float x, float x_max)
        {
                static_assert(std::is_signed<IntType>::value, "IntType must be signed");
                if (x_max <= 0.0f) { return static_cast<IntType>(0); }

                x = std::clamp(x, -x_max, x_max);

                const float t = x / x_max;
                constexpr float kMax = static_cast<float>(std::numeric_limits<IntType>::max());
                constexpr float kMin = static_cast<float>(std::numeric_limits<IntType>::min());

                return static_cast<IntType>(std::round(t >= 0.0f ? t * kMax : t * std::abs(kMin)));
        }

        // 协议 raw → 物理量
        template<typename IntType>
        float int_to_float(IntType x_int, float x_max)
        {
                static_assert(std::is_signed<IntType>::value, "IntType must be signed");

                constexpr float kMax = static_cast<float>(std::numeric_limits<IntType>::max());
                constexpr float kMin = static_cast<float>(std::numeric_limits<IntType>::min());

                if (x_int >= 0) { return static_cast<float>(x_int) / kMax * x_max; }
                return static_cast<float>(x_int) / kMin * x_max;
        }

        // 符号：>= 0 返回 +1（与 v2 的 sign() 语义一致）
        inline float sign(float v) { return (v >= 0.0f) ? 1.0f : -1.0f; }

        // 大端：DJI 协议里 16 位数都是高字节在前
        inline int16_t be_i16(const uint8_t *p)
        {
                return static_cast<int16_t>(
                        static_cast<uint16_t>(static_cast<uint16_t>(p[0]) << 8) |
                        static_cast<uint16_t>(p[1]));
        }

        inline void put_be_i16(uint8_t *p, int16_t v)
        {
                const uint16_t u = static_cast<uint16_t>(v);
                p[0] = static_cast<uint8_t>(u >> 8);
                p[1] = static_cast<uint8_t>(u & 0xFFU);
        }
} // namespace utils

#endif //THREE_FRICTION_UTILS_HPP
