/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define REJECT_THRESHOLD 0.0001f
#define DIAGONAL_CONSTANT 0.001f

#include <zephyr/kernel.h>
#include "mahalanobis_calculation.h"

// Calculates the pooled matrix of two matrices, it takes average of each element and creates a new matrix
void calculate_pooled_matrix(arm_matrix_instance_f32 *x_matrix, arm_matrix_instance_f32 *y_matrix, arm_matrix_instance_f32 *out_matrix, int num_rows, int num_cols)
{
    // TESTED OK
    for (uint16_t i = 0; i < num_rows; i++)
    {
        for (uint16_t j = 0; j < num_cols; j++)
        {
            out_matrix->pData[i * num_cols + j] = (x_matrix->pData[i * num_cols + j] + y_matrix->pData[i * num_cols + j]) / 2;
        }
    }
}

// Printing the matrix for debugging purposes
void print_matrix(arm_matrix_instance_f32 *matrix, int num_rows, int num_cols)
{
    // TESTED OK
    for (uint16_t i = 0; i < num_rows; i++)
    {
        for (uint16_t j = 0; j < num_cols; j++)
        {
            printk("%f ", matrix->pData[i * num_cols + j]);
        }
        printk("\n");
    }
}

// Calculates the covariance matrix of a matrix
arm_status calculate_covariance_matrix(arm_matrix_instance_f32 *matrix, int num_rows, int num_cols, float32_t *calculated_row_means, arm_matrix_instance_f32 *out_matrix)
{
    // TESTED OK
    // Subtract the mean from each row
    arm_status status;
    float32_t subtracted_data[num_rows * num_cols];
    arm_matrix_instance_f32 subtracted;
    arm_mat_init_f32(&subtracted, num_rows, num_cols, (float32_t *)subtracted_data);
    for (uint16_t i = 0; i < num_rows; i++)
    {
        for (uint16_t j = 0; j < num_cols; j++)
        {
            subtracted.pData[i * num_cols + j] = matrix->pData[i * num_cols + j] - calculated_row_means[i];
        }
    }
    float32_t transposed_data[num_rows * num_cols];
    arm_matrix_instance_f32 transposed;
    arm_mat_init_f32(&transposed, num_cols, num_rows, (float32_t *)transposed_data);
    status = arm_mat_trans_f32(&subtracted, &transposed);
    if (status != ARM_MATH_SUCCESS)
    {
        printk("Error covariance transpose\n");
        // Error handling
        return status;
    }
    status = arm_mat_mult_f32(&subtracted, &transposed, out_matrix);
    if (status != ARM_MATH_SUCCESS)
    {
        printk("Error covariance mat mul\n");
        // Error handling
        return status;
    }
    // divide by (num_cols-1)
    for (uint16_t i = 0; i < num_rows; i++)
    {
        for (uint16_t j = 0; j < num_rows; j++)
        {
            out_matrix->pData[i * num_rows + j] /= (num_cols - 1);
        }
    }
    return ARM_MATH_SUCCESS;
}

// Calculates the mean of each row of a matrix
void calculate_mean_of_rows(arm_matrix_instance_f32 *matrix, int num_rows, int num_cols, float32_t *row_means)
{
    // TESTED OK

    // Calculate the mean of each row
    for (uint16_t i = 0; i < num_rows; i++)
    {
        // Calculate the mean of the row
        arm_mean_f32((matrix->pData + i * num_cols), num_cols, &row_means[i]);
    }
}

// Calculates the mahalanobis distance between two matrices
arm_status calculate_mahalanobis(arm_matrix_instance_f32 *x_matrix, arm_matrix_instance_f32 *y_matrix, int num_rows, int num_cols, float32_t *out_result)
{

    arm_status status;

    // Calculate the mean of each row of each matrix and store it into array
    float32_t x_mean[num_rows];
    float32_t y_mean[num_rows];

    calculate_mean_of_rows(x_matrix, num_rows, num_cols, x_mean);
    calculate_mean_of_rows(y_matrix, num_rows, num_cols, y_mean);

    // Calculate the covariance matrix of each matrix, the calculation uses the means of each row
    float32_t x_covData[num_rows * num_rows];
    float32_t y_covData[num_rows * num_rows];

    arm_matrix_instance_f32 x_cov_matrix;
    arm_matrix_instance_f32 y_cov_matrix;

    arm_mat_init_f32(&x_cov_matrix, num_rows, num_rows, (float32_t *)x_covData);
    arm_mat_init_f32(&y_cov_matrix, num_rows, num_rows, (float32_t *)y_covData);

    status = calculate_covariance_matrix(x_matrix, num_rows, num_cols, x_mean, &x_cov_matrix);
    if (status != ARM_MATH_SUCCESS)
    {
        printk("Error covariance x\n");
        return status;
    }
    status = calculate_covariance_matrix(y_matrix, num_rows, num_cols, y_mean, &y_cov_matrix);
    if (status != ARM_MATH_SUCCESS)
    {
        printk("Error covariance y\n");
        return status;
    }

    // Calculate the pooled covariance matrix of the two matrices
    float32_t x_sub_y_means[num_rows];
    // we need the difference of the means of the two matrices for the pooled matrix
    arm_sub_f32(x_mean, y_mean, x_sub_y_means, num_rows);

    calculate_pooled_matrix(&x_cov_matrix, &y_cov_matrix, &y_cov_matrix, num_rows, num_rows);

    // Calculate the inverse of the pooled covariance matrix
    float32_t inv_covmatData[num_rows * num_rows];
    arm_matrix_instance_f32 inv_covmat;

    //    add small constant to the diagonal to avoid singularity
    for (uint16_t i = 0; i < num_rows; i++)
    {
        y_cov_matrix.pData[i * num_rows + i] += DIAGONAL_CONSTANT;
    }
    arm_mat_init_f32(&inv_covmat, num_rows, num_rows, (float32_t *)inv_covmatData);
    status = arm_mat_inverse_f32(&y_cov_matrix, &inv_covmat);
    if (status != ARM_MATH_SUCCESS)
    {
        printk("Error inverse\n");
        return status;
    }

    // Calculate the mahalanobis distance using the equation: sqrt((x-y)T * inv_covmat * (x-y))
    float32_t left_term[num_rows];
    arm_matrix_instance_f32 x_minus_y_matrix;
    arm_mat_init_f32(&x_minus_y_matrix, 1, num_rows, (float32_t *)x_sub_y_means);
    arm_matrix_instance_f32 left_term_matrix;
    arm_mat_init_f32(&left_term_matrix, num_rows, 1, (float32_t *)left_term);

    status = arm_mat_mult_f32(&x_minus_y_matrix, &inv_covmat, &left_term_matrix);
    if (status != ARM_MATH_SUCCESS)
    {
        printk("Error mult\n");
        return status;
    }
    arm_dot_prod_f32(left_term_matrix.pData, x_minus_y_matrix.pData, num_rows, out_result);
    status = arm_sqrt_f32(*out_result, out_result);
    return status;
}

// Normalizes the data of two matrices, the normalization is done by row, the data is normalized to the range [0,1]
// The normalization happens on both matrices, the same rows are normalized in both matrices.
// Features that have weird values (the difference between the max and min is less than a threshold) are rejected, other features are normalized and returned
uint8_t normalize_data(arm_matrix_instance_f32 *x_matrix, arm_matrix_instance_f32 *y_matrix, int num_rows, int num_cols)
{
    uint8_t num_of_valid_features = 0;
    uint8_t which_row_is_selected[num_rows];
    // TESTED OK
    float32_t matrix_min[num_rows], matrix_max[num_rows];
    // Calculate the max and min of each row combined across both matrices
    for (uint16_t i = 0; i < num_rows; i++)
    {
        float32_t tmp_max = x_matrix->pData[i * num_cols + 0];
        float32_t tmp_min = x_matrix->pData[i * num_cols + 0];
        for (uint16_t j = 0; j < num_cols; j++)
        {
            if (x_matrix->pData[i * num_cols + j] > tmp_max)
            {
                tmp_max = x_matrix->pData[i * num_cols + j];
            }
            if (x_matrix->pData[i * num_cols + j] < tmp_min)
            {
                tmp_min = x_matrix->pData[i * num_cols + j];
            }
        }
        for (uint16_t j = 0; j < num_cols; j++)
        {
            if (y_matrix->pData[i * num_cols + j] > tmp_max)
            {
                tmp_max = y_matrix->pData[i * num_cols + j];
            }
            if (y_matrix->pData[i * num_cols + j] < tmp_min)
            {
                tmp_min = y_matrix->pData[i * num_cols + j];
            }
        }
        // If the difference between the max and min is less than a threshold, reject the row (it is not a valid feature)
        if (tmp_max - tmp_min < REJECT_THRESHOLD)
        {
            continue;
        }
        matrix_max[num_of_valid_features] = tmp_max;
        matrix_min[num_of_valid_features] = tmp_min;
        which_row_is_selected[num_of_valid_features] = i;
        num_of_valid_features++;
    }
    // If there are less than 3 valid features, return -1, which means failure
    if (num_of_valid_features < 3)
    {
        return -1;
    }
    // Normalize the data of the selected rows and store it into the matrices
    for (uint16_t i = 0; i < num_of_valid_features; i++)
    {
        for (uint16_t j = 0; j < num_cols; j++)
        { // here into x_matrix and y_matrix in the first rows the data is written, so the data is normalized
            x_matrix->pData[i * num_cols + j] = (x_matrix->pData[which_row_is_selected[i] * num_cols + j] - matrix_min[i]) / (matrix_max[i] - matrix_min[i]);
            y_matrix->pData[i * num_cols + j] = (y_matrix->pData[which_row_is_selected[i] * num_cols + j] - matrix_min[i]) / (matrix_max[i] - matrix_min[i]);
        }
    }
    // Return the number of valid features
    return num_of_valid_features;
}

// Calculates the mahalanobis distance between two matrices and the threshold for the distance
arm_status estimate_difference_mahalanobis(arm_matrix_instance_f32 *x_matrix, arm_matrix_instance_f32 *y_matrix, int num_rows, int num_cols, float32_t *distance, float32_t *threshold)
{
    arm_status status;

    // Normalize the data of the two matrices and return the number of valid features, if there are less than 3 valid features, return error
    // Use only the remaining valid features for the calculation
    uint8_t num_of_valid_features;
    num_of_valid_features = normalize_data(x_matrix, y_matrix, num_rows, num_cols);
    printk("num_of_valid_features: %d\n", num_of_valid_features);
    if (num_of_valid_features < 3)
    {
        return ARM_MATH_LENGTH_ERROR;
    }
    num_rows = num_of_valid_features;

    // Calculate the mahalanobis distance between the two matrices
    status = calculate_mahalanobis(x_matrix, y_matrix, num_rows, num_cols, distance);
    if (status != ARM_MATH_SUCCESS)
    {
        printk("Error mahalanobis dist\n");
        return status;
    }

    // Calculate the threshold for the mahalanobis distance by splitting the first matrix into two halves and calculating the mahalanobis distance between the two halves

    // initializing the two halves
    float32_t first_half_x_data[num_rows * num_cols / 2];
    float32_t second_half_x_data[num_rows * num_cols / 2];
    arm_matrix_instance_f32 first_half_x;
    arm_matrix_instance_f32 second_half_x;
    arm_mat_init_f32(&first_half_x, num_rows, num_cols / 2, (float32_t *)first_half_x_data);
    arm_mat_init_f32(&second_half_x, num_rows, num_cols / 2, (float32_t *)second_half_x_data);

    // Split the first matrix into two halves
    for (uint16_t i = 0; i < num_rows; i++)
    {
        for (uint16_t j = 0; j < num_cols / 2; j++)
        {
            first_half_x.pData[i * num_cols / 2 + j] = x_matrix->pData[i * num_cols + j];
            second_half_x.pData[i * num_cols / 2 + j] = x_matrix->pData[i * num_cols + j + num_cols / 2];
        }
    }
    // Calculate the mahalanobis distance between the two halves to get threshold
    status = calculate_mahalanobis(&first_half_x, &second_half_x, num_rows, num_cols / 2, threshold);
    if (status != ARM_MATH_SUCCESS)
    {
        printk("Error mahalanobis th\n");
        return status;
    }
    return status;
}
