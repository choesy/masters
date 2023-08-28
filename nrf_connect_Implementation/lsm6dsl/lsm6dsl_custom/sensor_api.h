#ifndef LSM6DSL_SENSOR_API_H
#define LSM6DSL_SENSOR_API_H

/*
 ******************************************************************************
 * @file    imu_api.h
 * @author  Sensors Software Solution Team
 * @brief   LSM6DSL low level driver file
 ******************************************************************************
 */
/*
 * Copyright (c) 2021 Vincent Cergolj
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of IMU API.
 *
 * Author:          Vincent Cergolj
 */
#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include "lsm6dsl_reg.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @defgroup    IMU_API
     * @brief       This file provides a set of application specific interface functions to the
     *              lsm6dsl enhanced inertial module on NRF52.
     * @{
     *
     */

// extern stmdev_ctx_t* p_lsm6dsl_dev_ctx_t;

/** @brief Macro for defining size of the fifo buffer. */
#define IMU_FIFO_SIZE 256

/** Macro for defining sleep duration in activity/inactivity mode
 *           1 LSB corresponds to 512/ODR_XL
 */
#define IMU_SLEEP_DUR 1 /* For 52 Hz 1LSB = 9.84 s */

/** Macro for defining wake up threshold in activity/inactivity mode
 *           1 LSB = (FS_XL)/(2^6)  FS_XL as in full scale output eg. +- 2g
 */
#define IMU_WKUP_THRESHOLD 2 /* Treshold for the wake up interrupt For +-2 1LSB = 31.25 mg */

#define IMU_STEP_TRESHOLD 6           /*Treshold for detecting steps*/
#define IMU_TIME_BEFORE_STEP_RESET 18 /*Time betwen steps, so that the interrupt keeps counting them*/
#define IMU_STEP_DEBOUNCE_COUNT 7     /*number of debounce steps before step interrupt is triggerd*/

// Eeach LSB represents 5s. But it starts counting from 0. So that means for example 4-> 25 sec and not 20 sec.
#define STD_LENGHT_ARRAY_REFERENCE_PERIOD 4 /*Length of array to gather the reference data when the wristband is reset*/
#define STD_LENGTH_ARRAY 4                  /*Length of array for data calculations after the first inital referecne period*/

/** @brief Macro for selecting the number of axis we want to read. */
#define IMU_NUM_OF_AXIS 3 // SPREMENJENO IZ 6
/** @brief Macro for defining the size of the buffer array needed to store fifo data. */
#define IMU_BUFFER_SIZE (IMU_FIFO_SIZE * IMU_NUM_OF_AXIS) * 2 /*Multiplied by 2,                \
                                                                because imu outputs 16-bit data \
                                                                but we read 8-bit registers. */

    /**
     * \brief           This is a union that holds data from 3 axis
     * \note            This union is used to convert raw int data to to float
     */
    typedef union
    {
        int16_t i16bit[3];
        uint8_t u8bit[6];
    } axis3bit16_t;

    /**
     * \brief           This is imu data struct
     * \note            This structure is used to store converted imu data
     */
    typedef struct
    {
        float acc_x[IMU_FIFO_SIZE]; /*!< X axis accelerometer data */
        float acc_y[IMU_FIFO_SIZE]; /*!< Y axis accelerometer data */
        float acc_z[IMU_FIFO_SIZE]; /*!< Z axis accelerometer data */
        // float gyr_x[IMU_FIFO_SIZE]; /*!< X axis gyroscope data */
        // float gyr_y[IMU_FIFO_SIZE]; /*!< Y axis gyroscope data */
        // float gyr_z[IMU_FIFO_SIZE]; /*!< Z axis gyroscope data */
    } imu_data_t;

    extern imu_data_t imu_data;

    int32_t lsm6dsl_read_reg(const struct i2c_dt_spec *dev, uint8_t reg_addr,
                             uint8_t *data,
                             uint32_t len);

    int32_t lsm6dsl_write_reg(const struct i2c_dt_spec *dev, uint8_t reg_addr,
                              uint8_t *data,
                              uint32_t len);

    /**
     * \brief           Function that sets up activty/inactivity detection
     *
     * \param[in]       ctx: IMU structure from lsm6dsl library
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_activity_inactivity_setup(const struct i2c_dt_spec *dev);

    /**
     * \brief           Function that enables activty/inactivity detection
     *
     * \param[in]       ctx: IMU structure from lsm6dsl library
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_activity_inactivity_enable(const struct i2c_dt_spec *dev);

    /**
     * \brief           Function that disables activty/inactivity detection
     *
     * \param[in]       ctx: IMU structure from lsm6dsl library
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_activity_inactivity_disable(const struct i2c_dt_spec *dev);

    /**
     * \brief              Function that configures the device for absolute wrist
     *                     tilt detection.
     *
     * \param[in]          ctx:                 IMU structure from lsm6dsl library
     * \param[in]          latency_ms:          Angle detection latency in milliseconds
     *
     * \param[in]          wrist_titlt_mask:    Structure that holds the mask for selecting
     *                                          on what plane we want to detect the angle
     *
     * \param[in]          trigger_angle_deg:   Angle of detection in degrees
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_absolte_wrist_tilt_setup(const struct i2c_dt_spec *dev,
                                         uint16_t latency_ms,
                                         lsm6dsl_a_wrist_tilt_mask_t *wrist_tilt_mask,
                                         float trigger_angle_deg);
    /**
     * \brief           Function that disables fifo and sets up awt detection
     *
     * \param[in]       ctx: IMU structure from lsm6dsl library
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_awt_detection_mode(const struct i2c_dt_spec *dev);

    /**
     * @defgroup    IMU_API_fifo
     * @brief       This section provides all the functions concerning the fifo usage.
     * @{
     *
     */

    /**
     * \brief           Function disables the fifo buffer and turns off the gyroscope.
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_fifo_disable(const struct i2c_dt_spec *dev);
    /**
     * \brief           Function for reseting the fifo buffer
     *                  and preparing dma for the next reading.
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_fifo_reset();

    /**
     * \brief              Function for checking if fifo buffer is full.
     *
     * \param[in]          ctx: IMU structure from lsm6dsl library
     *
     * \retval true        FIFO buffer is full.
     * \retval false       FIFO buffer is not full.
     */
    uint8_t imu_is_fifo_full(const struct i2c_dt_spec *dev);

    /**
     * \brief           Function for enabling the device in fifo mode and routing
     *                  fifo full flag on INT2 pin of the device
     *
     * \param[in]       ctx:       IMU structure from lsm6dsl library
     * \param[in]       fifo_size: Size of the fifo buffer we want to configure
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_fifo_mode(const struct i2c_dt_spec *dev, uint16_t fifo_size);

    /**
     * \brief           Function for enabling the device in çontinious to fifo mode
     *
     * \param[in]       ctx:       IMU structure from lsm6dsl library
     * \param[in]       fifo_size: Size of the fifo buffer we want to configure
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_continious_to_fifo_setup(const struct i2c_dt_spec *dev, uint16_t fifo_size);

    /**
     * \brief           Function for reading and parsing the fifo data.
     *
     * \param[in]       ctx:       IMU structure from lsm6dsl library
     * \param[in]       data:      Pointer to imu_data_t struct
     *                             where parsed data is stored
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_read_fifo(const struct i2c_dt_spec *dev, imu_data_t *data);

    /**
     * \brief           Function for configuring the fifo buffer of the device
     *
     * \param[in]       ctx:       IMU structure from lsm6dsl library
     * \param[in]       fifo_size: Size of the fifo buffer we want to configure
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_fifo_setup(const struct i2c_dt_spec *dev, uint16_t fifo_size);

    /**
     * @}
     */

    /**
     * \brief           Function for checking if the device is present on the TWI bus
     *
     * \param[in]       ctx: IMU structure from lsm6dsl library
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_device_check(const struct i2c_dt_spec *dev);

    /**
     * \brief           Function for restoring the default settings of the device
     *
     * \param[in]       ctx: IMU structure from lsm6dsl library
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    uint8_t imu_restore_default_configuration(const struct i2c_dt_spec *dev);

    /**
     * \brief           Function for checking the fifo full flag and
     *                  setting imu modes accordingly
     *
     */
    void imu_handle_fifo_transfer_done();

    /**
     * \brief           Function for creating the features for drinking detection and
     *                  using the machine learning model for drinking predicitons.
     *
     * \param[in]       data:   Pointer to imu_data_t struct containing sensor samples
     * \param[in]       result: Pointer to an array of 2 elements holding the prediciton reusults
     *
     * \retval EXIT_SUCESS    Function completed sucessfully.
     */
    void imu_predict(imu_data_t *data, float *result);
    /**
     * \brief           Function for initializing the IMU device
     *
     * \param[in]       ctx: IMU structure from lsm6dsl library
     */
    int imu_init(const struct i2c_dt_spec *dev);

    /**
     * \brief           Function for handling events on push button of the wristband
     *
     */
    void imu_handle_push_button();
    /**
     * \brief           Function for handling events on INT2 pin of IMU
     *
     */
    void imu_handle_int2();
    /**
     * \brief           Function for handling events on INT1 pin of IMU
     *
     */
    void imu_handle_int1();
    /**
     * \brief           Function for reading fifo data and printing predicitons
     *
     */
    void imu_main_step_algorithm();

    void setup_gpio_interrupt();

    void imu_full_reset_dev();

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif // LSM6DSL_SENSOR_API_H
