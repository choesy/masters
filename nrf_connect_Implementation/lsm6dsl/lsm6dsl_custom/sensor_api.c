/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <features.h>
#include "sensor_api.h"
#include "mahalanobis_calculation.h"
#include "remote.h"

#define LOG_MODULE_NAME lsm6dsl
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

#define NUM_ROWS 35
#define NUM_COLS 118

K_SEM_DEFINE(imu_data_read_ready, 0, 1);
/** \brief   Global imu object */
const struct i2c_dt_spec *gdev;
/** \brief   Array used for storing converted fifo data */

/** \brief   Array used for storing fifo data */
uint8_t imu_data_buffer[IMU_BUFFER_SIZE];

int sample_index_count = 0;
float prediction_output = 0;

// set the interrupt struct to original state
void set_int2_to_zero(lsm6dsl_int2_route_t *int2)
{
	int2->int2_drdy_xl = 0;
	int2->int2_drdy_g = 0;
	int2->int2_drdy_temp = 0;
	int2->int2_fth = 0;
	int2->int2_fifo_ovr = 0;
	int2->int2_full_flag = 0;
	int2->int2_step_count_ov = 0;
	int2->int2_step_delta = 0;
	int2->int2_iron = 0;
	int2->int2_tilt = 0;
	int2->int2_6d = 0;
	int2->int2_double_tap = 0;
	int2->int2_ff = 0;
	int2->int2_wu = 0;
	int2->int2_single_tap = 0;
	int2->int2_inact_state = 0;
	int2->int2_wrist_tilt = 0;
}

// set the interrupt struct to original state
void set_int1_to_zero(lsm6dsl_int1_route_t *int1)
{
	int1->int1_drdy_xl = 0;
	int1->int1_drdy_g = 0;
	int1->int1_boot = 0;
	int1->int1_fth = 0;
	int1->int1_fifo_ovr = 0;
	int1->int1_full_flag = 0;
	int1->int1_sign_mot = 0;
	int1->int1_step_detector = 0;
	int1->int1_timer = 0;
	int1->int1_tilt = 0;
	int1->int1_6d = 0;
	int1->int1_double_tap = 0;
	int1->int1_ff = 0;
	int1->int1_wu = 0;
	int1->int1_single_tap = 0;
	int1->int1_inact_state = 0;
	int1->den_drdy_int1 = 0;
	int1->drdy_on_int1 = 0;
}

uint8_t imu_activity_inactivity_setup(const struct i2c_dt_spec *dev)
{
	lsm6dsl_xl_power_mode_set(dev, LSM6DSL_XL_NORMAL);
	/* Set Output Data Rate and full scale for accelerometer
	 * Disable gyroscope
	 **/
	lsm6dsl_xl_data_rate_set(dev, LSM6DSL_XL_ODR_52Hz);
	lsm6dsl_xl_full_scale_set(dev, LSM6DSL_2g);
	lsm6dsl_gy_data_rate_set(dev, LSM6DSL_GY_ODR_OFF);
	/* set duration for inactivity detection */
	lsm6dsl_act_sleep_dur_set(dev, IMU_SLEEP_DUR); /* Comes out as 10 seconds */

	/* Set Activity/Inactivity threshold */
	lsm6dsl_wkup_threshold_set(dev, IMU_WKUP_THRESHOLD); /* value is 2 which comes out as 62.5mg */

	/* Inactivity configuration: acc to 12.5 LP, gyro to Power-Down */
	lsm6dsl_act_mode_set(dev, LSM6DSL_XL_12Hz5_GY_PD);
	/* enable slope filter */
	lsm6dsl_xl_hp_path_internal_set(dev, LSM6DSL_USE_SLOPE);

	return EXIT_SUCCESS;
}
uint8_t imu_activity_inactivity_enable(const struct i2c_dt_spec *dev)
{
	return lsm6dsl_act_mode_set(dev, LSM6DSL_XL_12Hz5_GY_PD);
}

uint8_t imu_activity_inactivity_disable(const struct i2c_dt_spec *dev)
{
	return lsm6dsl_act_mode_set(dev, LSM6DSL_PROPERTY_DISABLE);
}

uint8_t imu_step_detection_setup(const struct i2c_dt_spec *dev)
{

	lsm6dsl_pedo_sens_set(dev, PROPERTY_ENABLE);
	k_msleep(50);
	lsm6dsl_pedo_sens_set(dev, PROPERTY_DISABLE);

	lsm6dsl_pedo_timeout_set(dev, IMU_TIME_BEFORE_STEP_RESET);	   // Koliko sekund med koraki, da je �e vedno zaznan korak
	lsm6dsl_pedo_threshold_set(dev, IMU_STEP_TRESHOLD);			   // Treshold da zazna korak
	lsm6dsl_pedo_debounce_steps_set(dev, IMU_STEP_DEBOUNCE_COUNT); // KOliko debounce korakov
	lsm6dsl_pedo_sens_set(dev, PROPERTY_ENABLE);

	lsm6dsl_data_ready_mode_set(dev, LSM6DSL_DRDY_PULSED);

	return EXIT_SUCCESS;
}

static uint8_t imu_fifo_reset_dev(const struct i2c_dt_spec *dev)
{
	lsm6dsl_fifo_mode_set(dev, LSM6DSL_BYPASS_MODE); /* resets buffer */
	lsm6dsl_fifo_mode_set(dev, LSM6DSL_FIFO_MODE);	 /* buffer starts */

	return EXIT_SUCCESS;
}

/// reset the device when the fifo is full
void imu_full_reset_dev()
{
	lsm6dsl_fifo_mode_set(gdev, LSM6DSL_BYPASS_MODE); /* resets buffer */
	lsm6dsl_fifo_mode_set(gdev, LSM6DSL_FIFO_MODE);	  /* buffer starts */
	uint8_t reset;
	lsm6dsl_reset_set(gdev, reset);
	do
	{
		lsm6dsl_reset_get(gdev, &reset);
	} while (reset);
	lsm6dsl_act_mode_set(gdev, LSM6DSL_XL_12Hz5_GY_PD);
}

// check if fifo is full
uint8_t imu_is_fifo_full(const struct i2c_dt_spec *dev)
{
	lsm6dsl_reg_t reg;
	int32_t ret;

	ret = lsm6dsl_read_reg(dev, LSM6DSL_FIFO_STATUS2, &reg.byte, 1);
	if (ret != 0)
	{
		LOG_INF("Error in imu_is_fifo_full function");
	}
	/* Check the fifo full smart flag in status register */
	if (reg.fifo_status2.fifo_full_smart == true)
		return 1;
	else
		return 0;
}

// cofigure the device to continiously write to fifo
uint8_t imu_continious_to_fifo_setup(const struct i2c_dt_spec *dev, uint16_t fifo_size)
{
	/*
	 *  Enable Block Data Update
	 */
	lsm6dsl_block_data_update_set(dev, PROPERTY_ENABLE);
	lsm6dsl_auto_increment_set(dev, PROPERTY_ENABLE);

	/* Before changing FIFO settings the device must be in bypass mode */
	lsm6dsl_fifo_mode_set(dev, LSM6DSL_BYPASS_MODE);

	/* Set power modes for accelerometer and gyroscope */
	lsm6dsl_xl_power_mode_set(dev, LSM6DSL_XL_NORMAL);
	lsm6dsl_gy_power_mode_set(dev, LSM6DSL_GY_NORMAL);

	/* Set Output Data Rate and full scale for accelerometer */
	lsm6dsl_xl_data_rate_set(dev, LSM6DSL_XL_ODR_52Hz);
	lsm6dsl_xl_full_scale_set(dev, LSM6DSL_2g);

	/* Set Output Data Rate and full scale for gyroscope */
	lsm6dsl_gy_data_rate_set(dev, LSM6DSL_GY_ODR_OFF); // SPREMENJEO IZ HZ
	lsm6dsl_gy_full_scale_set(dev, LSM6DSL_500dps);

	/* Set FIFO trigger - acc data ready signal */
	lsm6dsl_fifo_write_trigger_set(dev, LSM6DSL_TRG_XL_GY_DRDY);
	/* Set watermark level(size of FIFO) */
	lsm6dsl_fifo_watermark_set(dev, fifo_size * IMU_NUM_OF_AXIS + 3);
	/* Enable FIFO threshold */
	lsm6dsl_fifo_stop_on_wtm_set(dev, PROPERTY_ENABLE);

	/* Decimation - data to be stored in FIFO */
	lsm6dsl_fifo_gy_batch_set(dev, LSM6DSL_FIFO_GY_DISABLE); // SPREMENJENO IZ NEKI
	lsm6dsl_fifo_xl_batch_set(dev, LSM6DSL_FIFO_XL_NO_DEC);
	lsm6dsl_fifo_dataset_3_batch_set(dev, LSM6DSL_FIFO_DS3_DISABLE);
	lsm6dsl_fifo_dataset_4_batch_set(dev, LSM6DSL_FIFO_DS4_DISABLE);

	/* FIFO data rate and mode */
	lsm6dsl_fifo_data_rate_set(dev, LSM6DSL_FIFO_52Hz);
	lsm6dsl_fifo_mode_set(dev, LSM6DSL_BYPASS_MODE);
	lsm6dsl_fifo_mode_set(dev, LSM6DSL_STREAM_TO_FIFO_MODE);
	return EXIT_SUCCESS;
}

// check device id
uint8_t imu_device_check(const struct i2c_dt_spec *dev)
{
	/* Check device ID */
	uint8_t whoamI;
	lsm6dsl_device_id_get(dev, &whoamI);
	if (whoamI != LSM6DSL_ID)
	{
		LOG_INF("whoamI not matching");
		// while (1) ;
	}
	/* Restore default configuration */
	uint8_t reset;
	lsm6dsl_reset_set(dev, reset);
	do
	{
		lsm6dsl_reset_get(dev, &reset);
	} while (reset);

	return EXIT_SUCCESS;
}

// read fifo data and convert it to mg
uint8_t imu_read_fifo(const struct i2c_dt_spec *dev, imu_data_t *data)
{
	uint8_t data_width = IMU_NUM_OF_AXIS * 2;
	uint8_t data_raw_acceleration[data_width];

	for (uint16_t i = 0; i < IMU_FIFO_SIZE; i++)
	{

		memset(data_raw_acceleration, 0, data_width);
		lsm6dsl_fifo_raw_data_get(dev, data_raw_acceleration, data_width);

		data->acc_x[i] =

			lsm6dsl_from_fs2g_to_mg((data_raw_acceleration[1] << 8) | data_raw_acceleration[0]) / 1000;

		data->acc_y[i] =

			lsm6dsl_from_fs2g_to_mg((data_raw_acceleration[3] << 8) | data_raw_acceleration[2]) / 1000;

		data->acc_z[i] =

			lsm6dsl_from_fs2g_to_mg((data_raw_acceleration[5] << 8) | data_raw_acceleration[4]) / 1000;
	}
	imu_fifo_reset_dev(dev);

	return EXIT_SUCCESS;
}

volatile uint8_t activity_counter = 0;

volatile bool fifo_enable = 0;

static bool sleep_mode = false;
volatile bool enable_step_detection = true;

// handle the interrupt 2 from the device
void imu_handle_int2()
{

	LOG_INF("INTERRUPT 2");
	lsm6dsl_all_sources_t lsm6dsl_int_src;
	lsm6dsl_int1_route_t int1;
	lsm6dsl_int2_route_t int2;
	set_int1_to_zero(&int1);
	set_int2_to_zero(&int2);

	lsm6dsl_all_sources_get(gdev, &lsm6dsl_int_src);

	// when activity is detected
	if ((lsm6dsl_int_src.wake_up_src.wu_ia == 1) && (!fifo_enable))
	{
		enable_step_detection = true;
		int2.int2_full_flag = PROPERTY_DISABLE;
		int2.int2_inact_state = PROPERTY_ENABLE;
		int2.int2_wu = PROPERTY_DISABLE;
		int1.int1_step_detector = PROPERTY_ENABLE;
		lsm6dsl_pin_int1_route_set(gdev, int1);
		lsm6dsl_pin_int2_route_set(gdev, int2); // POMEMBEN VRSTNI RED
		imu_continious_to_fifo_setup(gdev, IMU_FIFO_SIZE);
		LOG_INF("wakeup");
	}

	// when inactivity is detected
	if (lsm6dsl_int_src.wake_up_src.sleep_state_ia == 1)
	{
		fifo_enable = false;
		sleep_mode = true;
		enable_step_detection = false;
		int2.int2_wu = PROPERTY_ENABLE;
		int2.int2_full_flag = PROPERTY_DISABLE;
		int2.int2_inact_state = PROPERTY_DISABLE;
		// int1.int1_drdy_xl=PROPERTY_DISABLE;
		// lsm6dsl_pin_int1_route_set(dev, int1);
		lsm6dsl_pin_int2_route_set(gdev, int2);
		lsm6dsl_fifo_mode_set(gdev, LSM6DSL_BYPASS_MODE);
		LOG_INF("SLEEP");
	}

	// when fifo is full
	if ((imu_is_fifo_full(gdev)) && (fifo_enable))
	{
		fifo_enable = false;
		activity_counter++;
		int2.int2_full_flag = PROPERTY_DISABLE;
		int2.int2_inact_state = PROPERTY_ENABLE;
		int2.int2_drdy_xl = PROPERTY_DISABLE;

		lsm6dsl_pin_int2_route_set(gdev, int2);
		k_sem_give(&imu_data_read_ready);
		LOG_INF("FIFO over");
	}
}

// handle the interrupt 1 from the device
void imu_handle_int1()
{
	// when step is detected
	if (enable_step_detection)
	{

		// lsm6dsl_int1_route_t int1;
		lsm6dsl_int2_route_t int2;
		// set_int1_to_zero(&int1);
		set_int2_to_zero(&int2);

		fifo_enable = true;
		enable_step_detection = false;
		int2.int2_full_flag = PROPERTY_ENABLE;
		int2.int2_inact_state = PROPERTY_ENABLE;
		int2.int2_wu = PROPERTY_DISABLE;
		// int1.int1_step_detector = PROPERTY_DISABLE;
		// lsm6dsl_pin_int1_route_set(dev, int1);
		lsm6dsl_pin_int2_route_set(gdev, int2);
		LOG_INF("STEP");
		// imu_activity_inactivity_disable(dev);
	}
}

volatile bool right_handed_person = true;
volatile bool print_hand_orientation = false;
void imu_handle_push_button()
{
	LOG_INF("BUTTON PRESSED");
}

int imu_init(const struct i2c_dt_spec *dev)
{
	gdev = dev;
	setup_gpio_interrupt();
	imu_device_check(dev);
	k_msleep(10);
	lsm6dsl_xl_power_mode_set(dev, LSM6DSL_XL_NORMAL);
	lsm6dsl_block_data_update_set(dev, PROPERTY_ENABLE);
	lsm6dsl_auto_increment_set(dev, PROPERTY_ENABLE);
	lsm6dsl_xl_lp1_bandwidth_set(dev, LSM6DSL_XL_LP1_ODR_DIV_4);
	lsm6dsl_xl_hp_bandwidth_set(dev, LSM6DSL_XL_HP_ODR_DIV_100);
	lsm6dsl_gy_band_pass_set(dev, LSM6DSL_HP_DISABLE_LP1_NORMAL);
	imu_activity_inactivity_setup(dev);
	imu_activity_inactivity_enable(dev);

	lsm6dsl_int1_route_t int1;
	set_int1_to_zero(&int1);
	lsm6dsl_int2_route_t int2;
	set_int2_to_zero(&int2);
	int2.int2_full_flag = PROPERTY_DISABLE;
	int2.int2_inact_state = PROPERTY_ENABLE;
	int2.int2_wu = PROPERTY_ENABLE;
	int1.int1_step_detector = PROPERTY_DISABLE;
	k_msleep(50);
	lsm6dsl_pin_int1_route_set(dev, int1);
	k_msleep(50);
	lsm6dsl_pin_int2_route_set(dev, int2); // VRSTNI RED POMBEMBEN PRI SETTANJU INT1 KER CENE SE DISEJBLA INTERRUPT
	imu_step_detection_setup(dev);

	return 0;
}

// THREAD
void imu_main_step_algorithm(struct bt_conn *conn)
{
	float32_t reference_features[NUM_ROWS * NUM_COLS];
	arm_matrix_instance_f32 reference_matrix;
	arm_mat_init_f32(&reference_matrix, NUM_ROWS, NUM_COLS, reference_features);
	float32_t new_features[NUM_ROWS * NUM_COLS];
	arm_matrix_instance_f32 new_matrix;
	arm_mat_init_f32(&new_matrix, NUM_ROWS, NUM_COLS, new_features);
	// first time variable is used when the reference matrix is filled, after it only the comparison matrix will get filled
	// reference matrix will stay the same
	bool first_time = true;
	arm_status status;

	// discard the very first measurement data from the beginnign, because of HIGHPASS FILTER
	bool verry_first_sample = true;

	while (1)
	{

		// wait for the fifo to be full, this semaphore is given in the interrupt handler
		k_sem_take(&imu_data_read_ready, K_FOREVER);
		imu_data_t imu_data;
		imu_read_fifo(gdev, &imu_data);
		enable_step_detection = true;
		// discard the very first measurement data from the beginnign, because of HIGHPASS FILTER
		if (verry_first_sample)
		{
			verry_first_sample = false;
			continue;
		}
		LOG_INF("FIFO READ");

		// calculate features from raw data
		float32_t features_arr[57];
		float32_t final_features_arr[35];
		CF_fill_features_arr(imu_data.acc_x, imu_data.acc_y, imu_data.acc_z, IMU_FIFO_SIZE, features_arr);

		// select the relevante features decided by analysis
		feature_selection(features_arr, final_features_arr);

		// PRINT FEATURES FOR DEBUG
		// for (int i = 0; i < 7; i++)
		// {
		// 	//k_sleep(K_MSEC(5));
		// 	send_gatt_notification_float(conn, final_features_arr + 5 * i, 5 * sizeof(float32_t));
		// }
		// continue;
		// PRINT RAW DATA FOR DEBUG
		// for (int i = 0; i < 51; i++)
		// {
		// 	// k_sleep(K_MSEC(5));
		// 	send_gatt_notification_float(conn, imu_data.acc_x + 5 * i, 5 * sizeof(float32_t));
		// }
		// continue;

		// fill the matrix with the data, each column is a sample and each row is a feature. The matrix is 35x118
		if (sample_index_count < NUM_COLS)
		{
			// fill the reference matrix with the first 118 samples if is the first time we ran the algorithm
			if (first_time)
			{
				for (int i = 0; i < NUM_ROWS; i++)
				{
					reference_matrix.pData[i * NUM_COLS + sample_index_count] = final_features_arr[i];
				}
			}
			else // Fill the new matrix with the next 118 samples (the comparison matrix)
			{
				for (int i = 0; i < NUM_ROWS; i++)
				{
					new_matrix.pData[i * NUM_COLS + sample_index_count] = final_features_arr[i];
				}
			}
			sample_index_count++;
			// send_gatt_notification_int(conn, &sample_index_count, sizeof(int));
			//  LOG_INF("count: %d", sample_index_count);
		}
		else
		{
			if (first_time)
			{
				first_time = false;
			}
			else
			{
				// if we have both matrices filled, we can calculate the mahalanobis distance
				float32_t distance_threshold[2];
				float32_t reference_copy[NUM_ROWS * NUM_COLS];
				arm_matrix_instance_f32 reference_matrix_copy;
				arm_mat_init_f32(&reference_matrix_copy, NUM_ROWS, NUM_COLS, reference_copy);
				memcpy(reference_matrix_copy.pData, reference_matrix.pData, NUM_ROWS * NUM_COLS * sizeof(float32_t));

				status = estimate_difference_mahalanobis(&reference_matrix_copy, &new_matrix, NUM_ROWS, NUM_COLS, &distance_threshold[0], &distance_threshold[1]);
				if (status != ARM_MATH_SUCCESS)
				{
					LOG_INF("Error in mahalanobis, status: %d", status);
				}
				else
				{
					// LOG_INF("----------------------------");
					// LOG_INF("distance: %f\n", distance_threshold[0]);
					// LOG_INF("threshold: %f\n", distance_threshold[1]);
					send_gatt_notification_float(conn, distance_threshold, 2 * sizeof(float32_t));
				}
			}
			sample_index_count = 0;
		}
	}
}

/*
0 abs mean x
1 std x
2 mean crossing rate x
3 sum per component x
4 iqr x
5 skewness x
6 kurtosis x
7 abs mean y
8 std y
9 mean crossing rate y
10 sum per component y
11 iqr y
12 skewness y
13 kurtosis y
14 abs mean z
15 std z
16 mean crossing rate z
17 sum per component z
18 iqr z
19 skewness z
20 kurtosis z
21 - 30 bins x
31 entropy_f x
32 skewnes_f x
33-42 bins y
43 entropy_f y
44 skewnes_f y
45-54 bins z
55 entropyfft z
56 skewnesfft z


*/