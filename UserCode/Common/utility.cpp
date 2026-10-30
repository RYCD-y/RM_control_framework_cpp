//
// Created by ye on 2026/10/30.
//

#include "utility.hpp"

template <typename IntType>
IntType float_to_signed(float x, float x_max) {
        static_assert(std::is_signed_v<IntType>, "IntType must be signed");

        // 1. 限幅
        x = std::clamp(x, -x_max, x_max);

        // 2. 归一化到 [-1.0, 1.0]
        float t = x / x_max;

        // 3. 映射到目标类型的范围
        constexpr auto max_val = static_cast<float>(std::numeric_limits<IntType>::max());
        constexpr auto min_val = static_cast<float>(std::numeric_limits<IntType>::min());

        if (t >= 0.0f)
                return static_cast<IntType>(std::round(t * max_val));
        else
                return static_cast<IntType>(std::round(t * std::abs(min_val)));
}

template <typename IntType>
float int16_to_float(IntType x_int, float x_max) {
        static_assert(std::is_signed_v<IntType>, "IntType must be signed");

        constexpr auto max_val = static_cast<float>(std::numeric_limits<IntType>::max());
        constexpr auto min_val = static_cast<float>(std::numeric_limits<IntType>::min());

        if (x_int >= 0)
                return static_cast<float>(x_int) / max_val * x_max;
        else
                return static_cast<float>(x_int) / min_val * x_max;
}