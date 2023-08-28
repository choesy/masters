#ifndef FEATURES_EXTRACT_H
#define FEATURES_EXTRACT_H

#include "arm_math.h"
#ifdef __cplusplus
extern "C"
{
#endif

    void CF_fill_features_arr(float *acc_X, float *acc_Y, float *acc_Z, uint16_t n_samples, float *features);
    void feature_selection(float32_t *features_arr, float32_t *features_arr_final);
#ifdef __cplusplus
}
#endif

#endif // FEATURES_EXTRACT_H
