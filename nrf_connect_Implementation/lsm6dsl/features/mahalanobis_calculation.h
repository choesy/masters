#ifndef MAHALANOBIS_CALCULATION_H
#define MAHALANOBIS_CALCULATION_H
#include "arm_math.h"
#ifdef __cplusplus
extern "C"
{
#endif
    arm_status estimate_difference_mahalanobis(arm_matrix_instance_f32 *x_matrix, arm_matrix_instance_f32 *y_matrix, int num_rows, int num_cols, float32_t *out_result, float32_t *threshold);
#ifdef __cplusplus
}
#endif

#endif // MAHALANOBIS_CALCULATION_H
