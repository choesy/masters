/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <zephyr/logging/log.h>
#include "features.h"
#include <zephyr/kernel.h>
#define LOG_MODULE_NAME features_extractor
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

// you should only change the CF_BUFFER_SIZE

#define CF_FS 52           // sampling frequency of the signal
#define CF_M 256           // window length
#define CF_D 128           // overlap length
#define CF_BUFFER_SIZE 256 // length of the array
#define CF_TIMESTEP 1 / CF_FS

// Hamming window for 255 samples
const float32_t window[] = {0.0, 0.00015059, 0.00060227, 0.00135477, 0.00240764, 0.00376023,
                            0.00541175, 0.00736118, 0.00960736, 0.01214894, 0.01498437, 0.01811197,
                            0.02152983, 0.02523591, 0.02922797, 0.0335036, 0.03806023, 0.04289512,
                            0.04800535, 0.05338785, 0.05903937, 0.0649565, 0.07113569, 0.07757322,
                            0.08426519, 0.09120759, 0.09839623, 0.10582679, 0.11349478, 0.12139558,
                            0.12952444, 0.13787647, 0.14644662, 0.15522973, 0.16422053, 0.17341357,
                            0.18280336, 0.1923842, 0.20215034, 0.2120959, 0.22221488, 0.2325012,
                            0.24294862, 0.25355092, 0.26430163, 0.27519435, 0.28622246, 0.29737934,
                            0.30865827, 0.32005247, 0.33155507, 0.34315914, 0.35485765, 0.3666436,
                            0.3785099, 0.39044937, 0.40245485, 0.41451904, 0.42663476, 0.43879467,
                            0.45099142, 0.4632177, 0.47546616, 0.4877294, 0.5, 0.5122706,
                            0.5245338, 0.53678226, 0.54900855, 0.5612053, 0.5733652, 0.5854809,
                            0.59754515, 0.6095506, 0.62149006, 0.6333564, 0.6451423, 0.65684086,
                            0.66844493, 0.6799475, 0.6913417, 0.7026207, 0.71377754, 0.72480565,
                            0.73569834, 0.7464491, 0.75705135, 0.7674988, 0.7777851, 0.7879041,
                            0.79784966, 0.8076158, 0.81719667, 0.8265864, 0.8357795, 0.84477025,
                            0.8535534, 0.86212355, 0.8704756, 0.8786044, 0.88650525, 0.8941732,
                            0.90160376, 0.90879244, 0.9157348, 0.92242676, 0.9288643, 0.9350435,
                            0.94096065, 0.9466122, 0.95199466, 0.95710486, 0.96193975, 0.9664964,
                            0.970772, 0.9747641, 0.97847015, 0.98188806, 0.98501563, 0.9878511,
                            0.9903926, 0.9926388, 0.99458826, 0.9962398, 0.9975924, 0.99864525,
                            0.99939775, 0.99984944, 1.0, 0.99984944, 0.99939775, 0.99864525,
                            0.9975924, 0.9962398, 0.99458826, 0.9926388, 0.9903926, 0.9878511,
                            0.98501563, 0.98188806, 0.97847015, 0.9747641, 0.970772, 0.9664964,
                            0.96193975, 0.95710486, 0.95199466, 0.9466122, 0.94096065, 0.9350435,
                            0.9288643, 0.92242676, 0.9157348, 0.90879244, 0.90160376, 0.8941732,
                            0.88650525, 0.8786044, 0.8704756, 0.86212355, 0.8535534, 0.84477025,
                            0.8357795, 0.8265864, 0.81719667, 0.8076158, 0.79784966, 0.7879041,
                            0.7777851, 0.7674988, 0.75705135, 0.7464491, 0.73569834, 0.72480565,
                            0.71377754, 0.7026207, 0.6913417, 0.6799475, 0.66844493, 0.65684086,
                            0.6451423, 0.6333564, 0.62149006, 0.6095506, 0.59754515, 0.5854809,
                            0.5733652, 0.5612053, 0.54900855, 0.53678226, 0.5245338, 0.5122706,
                            0.5, 0.4877294, 0.47546616, 0.4632177, 0.45099142, 0.43879467,
                            0.42663476, 0.41451904, 0.40245485, 0.39044937, 0.3785099, 0.3666436,
                            0.35485765, 0.34315914, 0.33155507, 0.32005247, 0.30865827, 0.29737934,
                            0.28622246, 0.27519435, 0.26430163, 0.25355092, 0.24294862, 0.2325012,
                            0.22221488, 0.2120959, 0.20215034, 0.1923842, 0.18280336, 0.17341357,
                            0.16422053, 0.15522973, 0.14644662, 0.13787647, 0.12952444, 0.12139558,
                            0.11349478, 0.10582679, 0.09839623, 0.09120759, 0.08426519, 0.07757322,
                            0.07113569, 0.0649565, 0.05903937, 0.05338785, 0.04800535, 0.04289512,
                            0.03806023, 0.0335036, 0.02922797, 0.02523591, 0.02152983, 0.01811197,
                            0.01498437, 0.01214894, 0.00960736, 0.00736118, 0.00541175, 0.00376023,
                            0.00240764, 0.00135477, 0.00060227, 0.00015059};

#define SCALE 1 / (CF_FS * 96)
// calculates mean value
// takes in array and its size
float CF_mean_value(float *arr, int n)
{
    float sum = 0;
    for (int i = 0; i < n; i++)
        sum = sum + arr[i];
    return sum / n;
}

// calculate standard deviation
// takes in array and its size
float CF_standard_deviation(float *arr, int n)
{
    float sum = 0;
    float meanVal = CF_mean_value(arr, n);

    for (int i = 0; i < n; i++)
        sum = sum + (arr[i] - meanVal) *
                        (arr[i] - meanVal);
    return sqrt(sum / (n));
}

// takes in arrray and its size
// function calls standard_deviation and mean_value
float CF_coefficient_of_variation(float *arr, int n)
{
    return CF_standard_deviation(arr, n) / CF_mean_value(arr, n);
}

/*number of zero crossings*/
// if you multiply two adjacent values and have a negative result it means you have crossed the 0 line
int CF_num_of_zero_crossings(float *arr, int n)
{
    int count = 0;
    for (int i = 1; i < n; i++)
    {
        if (arr[i] * arr[i - 1] < 0)
            count++;
    }
    return count;
}

/* amplitude of a signal */
// finds max and min value and subtracts them
float CF_calc_amplitude(float *arr, int n)
{
    float max = arr[0];
    float min = arr[0];

    for (int i = 0; i < n; i++)
    {
        if (arr[i] > max)
            max = arr[i];
        if (arr[i] < min)
            min = arr[i];
    }
    return max - min;
}

/* velocity*/
// rewritten in c-code from notes
// TODO: maybe odstrain
float CF_comput_AC_velocity(float *arr, int n, float timestep)
{
    float calc = 0;

    for (int i = 0; i < n; i++)
    {
        calc += arr[i];
    }
    return calc * timestep;
}
/*calculate sum per component*/
// rewritten in c-code from notes
// TODO: maybe odstrain
float CF_calc_sum_per_component(float32_t *arr, int n, float32_t timestep)
{
    float calc = 0;

    for (int i = 0; i < n; i++)
    {
        calc += fabs(arr[i]);
    }
    return calc * timestep;
}
/*absolute mean value */
// just calculate mean value and take the absolute of it
float32_t CF_abs_mean_value(float32_t *arr, uint32_t n)
{
    float32_t sum = 0;
    float32_t temp = 0;
    for (int32_t i = 0; i < n; i++)
    {
        temp = *(arr + i);
        sum += fabs(temp);
    }
    return (float)sum / (float)n;
}
float32_t CF_calc_entropy(float32_t *arr, uint32_t num_symbols)
{ // dodadi normalizacija na nizata p = 1.0*p/suma od p i na kraj bez logaritmot i OK ke e
    float32_t entropy = 0;
    float32_t sum = 0;
    for (int16_t i = 0; i < num_symbols; i++)
    {
        sum += *(arr + i);
    }
    float32_t temp = 0;

    for (int16_t i = 0; i < num_symbols; i++)
    {
        temp = arr[i] / sum;

        if (temp > 0.0)
        {
            entropy -= temp * (float32_t)log((float32_t)temp);
        }
    }

    // entropy = entropy/ (float) log ((float) 2.0);
    return entropy;
}
// TODO: maybe odstrain
int CF_calc_mean_crossing_rate(float *arr, int n)
{
    /// ALI JE PRAVA TA FUNKCIJA, PREVERI SE NAVADEN MEAN VALUE
    float mean = CF_abs_mean_value(arr, n); // using abs mean because in kotlin there was a math.abs function used

    int crossings = 0;
    float last = arr[0] - mean;
    float current = 0;

    for (int i = 1; i < n; i++)
    {
        current = arr[i] - mean;
        if (last * current < 0)
            crossings++;
        last = current;
    }
    return crossings;
}
// support function for qsort()
int CF_cmpfunc(const void *a, const void *b)
{
    return (*(float *)a >= *(float *)b) ? 1 : -1;
    // return (  *(int*)a - *(int*)b);
}
// Function to give index of the median
int32_t median(float32_t *a, int32_t l, int32_t r)
{
    int32_t n = r - l + 1;
    n = (n + 1) / 2 - 1;
    return n + l;
}

// Function to calculate IQR
float32_t CF_IQR(float32_t *a, uint32_t n)
{
    float32_t temp_arr[n];
    // store array in temp_array to preserve it

    for (uint32_t i = 0; i < n; i++)
    {
        *(temp_arr + i) = *(a + i);
    }
    // sort the temp_arr
    qsort(temp_arr, n, sizeof(float32_t), CF_cmpfunc);

    // Index of median of entire data
    int32_t mid_index = median(temp_arr, 0, n);
    int32_t second_index = 0;
    // Median of first half
    float32_t Q1 = temp_arr[median(temp_arr, 0, mid_index)];

    // Median of second half
    second_index = /* mid_index + */ median(temp_arr, mid_index + 1, n); // preserving original code that worked weird
    float32_t Q3 = temp_arr[second_index];

    // IQR calculation
    return (Q3 - Q1);
}
// summarize squared values of arr
float32_t CF_total_fft_sum(float32_t *arr, uint16_t n)
{
    float32_t sum = 0;
    for (uint16_t i = 0; i < n; i++)
    {
        sum += *(arr + i) * *(arr + i);
    }
    return sum;
}

// Calculate skewness and kurtosis
void CF_func_skewness_and_kurtosis(float32_t *niza, uint32_t n, float32_t *skewness, float32_t *kurtosis)
{
    float32_t mean = 0.0;
    float32_t m2 = 0.0;
    float32_t m3 = 0.0;
    float32_t m4 = 0.0;
    uint32_t i;

    *skewness = 0.0;
    *kurtosis = 0.0;

    for (i = 0; i < n; i++)
    {
        mean += *(niza + i);
    }
    mean = mean / n;
    // We calculated mean value, next we will calculate second and third moment around the mean
    for (i = 0; i < n; i++)
    {
        m2 += (*(niza + i) - mean) * (*(niza + i) - mean);
        m3 += (*(niza + i) - mean) * (*(niza + i) - mean) * (*(niza + i) - mean);
        m4 += (*(niza + i) - mean) * (*(niza + i) - mean) * (*(niza + i) - mean) * (*(niza + i) - mean);
    }
    m2 = m2 / (float32_t)n;
    m3 = m3 / (float32_t)n;
    m4 = m4 / (float32_t)n;
    *skewness = m3 / pow(m2, 1.5);
    *kurtosis = m4 / pow(m2, 2);
    *kurtosis = *kurtosis - 3;
}
// multiply the input array with the window
void CF_window_data(float32_t *niza, uint32_t n, const float32_t *win_values)
{
    uint32_t i = 0;
    for (i = 0; i < n; i++)
    {
        *(niza + i) = *(niza + i) * *(win_values + i);
    }
}
void CF_detrend_func(float32_t *niza, uint32_t n)
{
    float32_t sum = 0.0;
    float32_t mean = 0.0;
    int32_t i = 0;

    for (i = 0; i < n; i++)
    {
        sum += *(niza + i);
    }

    mean = sum / n;

    for (i = 0; i < n; i++)
    {
        *(niza + i) = *(niza + i) - mean;
    }
}

// function for fft
void CF_fft(float32_t *input_arr, uint32_t n, float32_t *output_arr)
{

    float32_t temp_arr_in[n], temp_arr_out[n];
    arm_rfft_fast_instance_f32 varInstRfftF32;
    arm_status status;
    // fill the temp arr with input arr values
    for (uint32_t i = 0; i < n; i++)
        temp_arr_in[i] = *(input_arr + i);
    // CF_detrend_func(temp_arr_in, n); //already having the highpass
    CF_window_data(temp_arr_in, CF_M, window);
    status = arm_rfft_fast_init_f32(&varInstRfftF32, n);

    arm_rfft_fast_f32(&varInstRfftF32, temp_arr_in, temp_arr_out, 0);

    float32_t temp;
    temp = *(temp_arr_out + 1); // we need to save the second element
    *(temp_arr_out + 1) = 0;    // set second element to 0 because the DC component of fft can't have an imaginary part

    // moving the second element to the middle of output array and setting the +1 element to 0
    *(output_arr + n - 1) = temp;

    for (uint32_t i = 0; i < n; i += 1)
    {
        // fil the output_arr with input values
        *(output_arr + i) = *(temp_arr_out + i);
    }
}

// function filling temporal features array
void CF_fill_raw_features(float32_t *features, float32_t *input_arr, uint16_t n)
{
    features[0] = CF_abs_mean_value(input_arr, n);
    features[1] = CF_standard_deviation(input_arr, n);
    features[2] = CF_calc_mean_crossing_rate(input_arr, n);
    features[3] = CF_calc_sum_per_component(input_arr, n, (float32_t)CF_TIMESTEP);
    features[4] = CF_IQR(input_arr, n);
    CF_func_skewness_and_kurtosis(input_arr, n, &features[5], &features[6]);
}

// function for absolute value and energy
void CF_absolute_value_complex(float32_t *niza, uint32_t n, float32_t *energy_feat, float32_t *entropy)
{

    float32_t absolute_array[n / 2]; // array half the size of input array to store calculated absolute values
    float32_t sum = 0.0;             // In this variable we will have sum of square of absolute values
    int32_t counter = 0;
    for (uint32_t i = 0; i < n; i += 2)
    {
        float re, img, abs_value;
        re = niza[i];
        img = niza[i + 1];

        // abs_value = sqrt(re * re + img * img);
        abs_value = re * re + img * img;
        absolute_array[counter] = abs_value;
        counter++;

        // abs_value = abs_value * abs_value;
        sum += abs_value;
    }

    *energy_feat = sum / n;
    *entropy = CF_calc_entropy(absolute_array, n / 2);
}

// my function for psd, it takes in the fft result and the length of the fft result
void PSD(float32_t *fft_result, uint32_t n, float32_t *output_array) // output array needs to be length n/2 + 1 so in my case 129
{

    arm_rfft_fast_instance_f32 fft_inst;  // FFT instance
    arm_rfft_fast_init_f32(&fft_inst, n); // Initialize FFT instance

    // Calculate PSD
    output_array[0] = fft_result[0] * fft_result[0];     // DC component
    output_array[n / 2] = fft_result[1] * fft_result[1]; // Nyquist frequency component
    for (uint32_t i = 1; i < n / 2; i++)
    { // Other frequency components
        float real_part = fft_result[2 * i];
        float imag_part = fft_result[2 * i + 1];
        output_array[i] = real_part * real_part + imag_part * imag_part;
    }
}

// function for calculating the sum of the first 5 bins
float32_t normalized_squared_sum_N_elements(float32_t *arr, uint16_t n, uint16_t start, uint16_t stop, float32_t total_fft_sum)
{
    float32_t sum = 0;
    for (uint16_t i = start; i < stop; i++)
    {
        sum += *(arr + i) * *(arr + i);
    }
    return sum / total_fft_sum;
}

// function for filling the bin array
void CF_fill_bin_arr(float32_t *bin_arr, float32_t *input_arr, float32_t fft_sum)
{

    for (uint16_t i = 0; i < 45; i += 5)
    {

        *(bin_arr + i / 5) = normalized_squared_sum_N_elements(input_arr, 128, i, i + 5, fft_sum);
    }

    *(bin_arr + 9) = normalized_squared_sum_N_elements(input_arr, 128, 45, 128, fft_sum);
}

// function for calculating the frequency values
void CF_sample_frequencies(float32_t *niza, uint32_t n, int32_t nfft, int32_t fs)
{

    int32_t i;
    for (i = 0; i < n; i++)
    {
        niza[i] = (float)(i * fs) / nfft;
    }
}

// function for finding the max value in an array
void CF_find_N_max_values(float32_t *input_arr, uint32_t arr_len, uint32_t n, float32_t *result_values, int16_t *result_indicies)
{
    float32_t temp_arr[arr_len];
    float32_t pResult = 0;
    uint32_t pIndex = 0;
    // store array in temp_array to preserve it
    for (uint32_t i = 0; i < arr_len; i++)
        *(temp_arr + i) = *(input_arr + i);

    // finds the max value and puts it to -2147483648(smallest int value) so we can get the next biggest value
    for (uint32_t i = 0; i < n; i++)
    {
        arm_max_f32(temp_arr, arr_len, &pResult, &pIndex);
        *(result_values + i) = pResult;
        *(result_indicies + i) = pIndex;
        *(temp_arr + pIndex) = FLT_MIN;
    }
}

// function for filling the frequency features array
void CF_fill_frequency_features(float32_t *features, float32_t *input_arr, uint16_t n)
{
    float32_t bin_values[10], Pxx_den[128], regularFFT[256];
    float32_t energy = 0, entropy = 0, skewness = 0, kurtosis = 0;

    CF_fft(input_arr, n, regularFFT);

    CF_absolute_value_complex(regularFFT, n, &energy, &entropy); // calculates energy and entropy
    PSD(regularFFT, n, Pxx_den);
    float fft_sum = CF_total_fft_sum(Pxx_den, 128);
    CF_fill_bin_arr(bin_values, Pxx_den, fft_sum);
    CF_func_skewness_and_kurtosis(Pxx_den, 128, &skewness, &kurtosis);

    for (int i = 0; i < 10; i++)
    {
        features[i] = bin_values[i];
    }
    features[10] = entropy;
    features[11] = skewness;
}

// function for filling all the features array
void CF_fill_features_arr(float *acc_X, float *acc_Y, float *acc_Z, uint16_t n_samples, float32_t *features)
{
    float *axis[3] = {acc_X, acc_Y, acc_Z};

    for (uint16_t i = 0; i < 3; i++)
    {
        CF_fill_raw_features(&features[i * 7], axis[i], n_samples);
    }
    for (uint16_t i = 0; i < 3; i++)
    {
        CF_fill_frequency_features(&features[i * 12 + 21], axis[i], n_samples);
    }
}

// selecting only the features that are relevant for the model
void feature_selection(float32_t *features_arr, float32_t *final_features_arr)
{
    final_features_arr[0] = features_arr[0];   // abs mean x
    final_features_arr[1] = features_arr[1];   // std x
    final_features_arr[2] = features_arr[2];   // mean crossing rate x
    final_features_arr[3] = features_arr[3];   // sum per component x  NOOOOOOO 0
    final_features_arr[4] = features_arr[4];   // iqr x
    final_features_arr[5] = features_arr[6];   // kurtosis x
    final_features_arr[6] = features_arr[10];  // sum per component y
    final_features_arr[7] = features_arr[11];  // iqr y
    final_features_arr[8] = features_arr[14];  // abs mean z
    final_features_arr[9] = features_arr[15];  // std z NOOOOOOO 0
    final_features_arr[10] = features_arr[16]; // mean crossing rate z
    final_features_arr[11] = features_arr[17]; // sum per component z
    final_features_arr[12] = features_arr[18]; // iqr z
    final_features_arr[13] = features_arr[19]; // skewness z
    final_features_arr[14] = features_arr[21]; // bins x 1
    final_features_arr[15] = features_arr[22]; // bins x 2
    final_features_arr[16] = features_arr[23]; // bins x 3
    final_features_arr[17] = features_arr[24]; // bins x 4
    final_features_arr[18] = features_arr[25]; // bins x 5
    final_features_arr[19] = features_arr[26]; // bins x 6
    final_features_arr[20] = features_arr[28]; // bins x 8
    final_features_arr[21] = features_arr[29]; // bins x 9
    final_features_arr[22] = features_arr[31]; // entropy_f x NOOOO
    final_features_arr[23] = features_arr[32]; // skewnes_f x
    final_features_arr[24] = features_arr[33]; // bins y 1
    final_features_arr[25] = features_arr[34]; // bins y 2
    final_features_arr[26] = features_arr[35]; // bins y 3
    final_features_arr[27] = features_arr[36]; // bins y 4
    final_features_arr[28] = features_arr[37]; // bins y 5
    final_features_arr[29] = features_arr[38]; // bins y 6
    final_features_arr[30] = features_arr[43]; // entropy_f y NONE
    final_features_arr[31] = features_arr[45]; // bins z 1
    final_features_arr[32] = features_arr[46]; // bins z 2
    final_features_arr[33] = features_arr[47]; // bins z 3
    final_features_arr[34] = features_arr[55]; // entropy_f z NONE
}