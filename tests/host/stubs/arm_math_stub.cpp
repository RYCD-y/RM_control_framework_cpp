// host 替身实现：与 CMSIS 相同的"行主序 + i/k/j 累加顺序"，便于与目标机 bit-exact
#include <cmath>
#include <cstring>

#include "arm_math.h"

extern "C" {

arm_status arm_mat_init_f32(arm_matrix_instance_f32* S, uint16_t nRows, uint16_t nColumns,
                            float32_t* pData) {
    if (S == nullptr) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    S->numRows = nRows;
    S->numCols = nColumns;
    S->pData = pData;
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32* A, const arm_matrix_instance_f32* B,
                           arm_matrix_instance_f32* D) {
    if (!A || !B || !D) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    if (A->numRows != B->numRows || A->numCols != B->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    if (A->numRows != D->numRows || A->numCols != D->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    for (uint32_t i = 0; i < (uint32_t)A->numRows * A->numCols; i++) {
        D->pData[i] = A->pData[i] + B->pData[i];
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32* A, const arm_matrix_instance_f32* B,
                           arm_matrix_instance_f32* D) {
    if (!A || !B || !D) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    if (A->numRows != B->numRows || A->numCols != B->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    if (A->numRows != D->numRows || A->numCols != D->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    for (uint32_t i = 0; i < (uint32_t)A->numRows * A->numCols; i++) {
        D->pData[i] = A->pData[i] - B->pData[i];
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32* A, const arm_matrix_instance_f32* B,
                            arm_matrix_instance_f32* D) {
    if (!A || !B || !D) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    if (A->numCols != B->numRows) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    if (D->numRows != A->numRows || D->numCols != B->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    for (uint16_t r = 0; r < A->numRows; r++) {
        for (uint16_t c = 0; c < B->numCols; c++) {
            float32_t sum = 0.0f;
            for (uint16_t k = 0; k < A->numCols; k++) {
                sum += A->pData[r * A->numCols + k] * B->pData[k * B->numCols + c];
            }
            D->pData[r * D->numCols + c] = sum;
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* D) {
    if (!A || !D) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    if (D->numRows != A->numCols || D->numCols != A->numRows) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    for (uint16_t r = 0; r < A->numRows; r++) {
        for (uint16_t c = 0; c < A->numCols; c++) {
            D->pData[c * D->numCols + r] = A->pData[r * A->numCols + c];
        }
    }
    return ARM_MATH_SUCCESS;
}

// 高斯-约当消元（带部分选主元），与 CMSIS 的 LU 实现结果等价
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* D) {
    if (!A || !D || A->numRows != A->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    const uint16_t n = A->numRows;
    if (D->numRows != n || D->numCols != n) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    static float32_t aug[16][32];  // host 测试最大 16 阶（EKF 用 7 阶）
    if (n > 16) {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    for (uint16_t r = 0; r < n; r++) {
        for (uint16_t c = 0; c < n; c++) {
            aug[r][c] = A->pData[r * n + c];
        }
        for (uint16_t c = 0; c < n; c++) {
            aug[r][n + c] = (r == c) ? 1.0f : 0.0f;
        }
    }

    for (uint16_t col = 0; col < n; col++) {
        uint16_t piv = col;
        float best = std::fabs(aug[col][col]);
        for (uint16_t r = col + 1; r < n; r++) {
            if (std::fabs(aug[r][col]) > best) {
                best = std::fabs(aug[r][col]);
                piv = r;
            }
        }
        if (best < 1e-12f) {
            return ARM_MATH_SINGULAR;
        }

        if (piv != col) {
            for (uint16_t c = 0; c < 2 * n; c++) {
                const float t = aug[col][c];
                aug[col][c] = aug[piv][c];
                aug[piv][c] = t;
            }
        }

        const float inv = 1.0f / aug[col][col];
        for (uint16_t c = 0; c < 2 * n; c++) {
            aug[col][c] *= inv;
        }

        for (uint16_t r = 0; r < n; r++) {
            if (r == col) {
                continue;
            }
            const float f = aug[r][col];
            if (f == 0.0f) {
                continue;
            }
            for (uint16_t c = 0; c < 2 * n; c++) {
                aug[r][c] -= f * aug[col][c];
            }
        }
    }

    for (uint16_t r = 0; r < n; r++) {
        for (uint16_t c = 0; c < n; c++) {
            D->pData[r * n + c] = aug[r][n + c];
        }
    }
    return ARM_MATH_SUCCESS;
}

float32_t arm_cos_f32(float32_t x) { return std::cos(x); }
float32_t arm_sin_f32(float32_t x) { return std::sin(x); }

}  // extern "C"
