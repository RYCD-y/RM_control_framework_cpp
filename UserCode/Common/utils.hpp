//
// Created by ye on 2026/10/30.
//

#ifndef THREE_FRICTION_UTILS_HPP
#define THREE_FRICTION_UTILS_HPP

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <sys/_stdint.h>

namespace utils
{
        template<typename IntType>
        IntType float_to_signed(float x, float x_max);

        template<typename IntType>
        float int_to_float(IntType x_int, float x_max);
}

#endif //THREE_FRICTION_UTILS_HPP
