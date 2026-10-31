// host 替身：CMSIS-DSP 里 kalman_filter 用到的那一小部分矩阵接口
#ifndef ARM_MATH_HOST_STUB_H
#define ARM_MATH_HOST_STUB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef float float32_t;

typedef enum
{
        ARM_MATH_SUCCESS        = 0,
        ARM_MATH_ARGUMENT_ERROR = 1,
        ARM_MATH_LENGTH_ERROR   = 2,
        ARM_MATH_SIZE_MISMATCH  = 3,
        ARM_MATH_NANINF         = 4,
        ARM_MATH_SINGULAR       = 5
} arm_status;

typedef struct
{
        uint16_t numRows;
        uint16_t numCols;
        float32_t *pData;
} arm_matrix_instance_f32;

arm_status arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nColumns, float32_t *pData);
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst);
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst);

float32_t arm_cos_f32(float32_t x);
float32_t arm_sin_f32(float32_t x);

#ifdef __cplusplus
}
#endif

#endif
