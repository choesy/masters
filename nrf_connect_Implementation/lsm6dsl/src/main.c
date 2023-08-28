/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <sensor_api.h>
#include <remote.h>
#include <zephyr/sys/reboot.h>

#define LOG_MODULE_NAME main

#define LSM6DSL_REG_ADDRESS 0x6a

#define I2C_DEV_NODE DT_NODELABEL(i2c1)
#define PRIORITY 5

LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);
static struct bt_conn *current_conn;
K_SEM_DEFINE(bt_is_connected, 0, 1);
/* Declarations */
void on_connected(struct bt_conn *conn, uint8_t err);
void on_disconnected(struct bt_conn *conn, uint8_t reason);
void on_notif_changed(bool status);
void on_data_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len);

struct bt_remote_service_cb remote_cb = {
	.notif_changed = on_notif_changed,
	.data_received = on_data_received,
};

struct bt_conn_cb bluetooth_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};
/* Callbacks */
void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		LOG_ERR("connection err: %d", err);
		return;
	}
	LOG_INF("Connected.");
	current_conn = bt_conn_ref(conn);
	k_sem_give(&bt_is_connected);
	// dk_set_led_on(CONN_STATUS_LED);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason: %d)", reason);
	// dk_set_led_off(CONN_STATUS_LED);
	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
		// when disconnected, reset the device
		imu_full_reset_dev();
		sys_reboot(SYS_REBOOT_COLD);
	}
}

void on_notif_changed(bool status)
{
	// RX stack must be 2048 bytes or more
	if (status == 1) // THIS CRASHES THE PROGRAM
	{
		LOG_INF("Notifications enabled");
	}
	else
	{
		LOG_INF("Notifications disabled");
	}
}

void on_data_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	uint8_t temp_str[len + 1];
	memcpy(temp_str, data, len);
	temp_str[len] = 0x00;

	LOG_INF("Received data on conn %p. Len: %d", (void *)conn, len);
	LOG_INF("Data: %s", temp_str);
}

void main(void)
{
	LOG_INF("STARTTTING\n");
	int err;

	// first step is to initialize the bluetooth stack
	err = bluetooth_init(&bluetooth_callbacks, &remote_cb);
	if (err)
	{
		LOG_ERR("bluetooth_init failed (err %d)", err);
		return;
	}
	// wait for the connection to be established with the phone, this is a blocking call
	k_sem_take(&bt_is_connected, K_FOREVER);

	// when the phone is connected, we can start the rest of the program

	// initialize the I2C device
	const struct device *const i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);
	if (i2c_dev == NULL || !device_is_ready(i2c_dev))
	{
		LOG_INF("Could not get I2C device\n");
		return;
	}
	const struct i2c_dt_spec spec = {
		.bus = i2c_dev,
		.addr = LSM6DSL_REG_ADDRESS};

	// check if the LSM6DSL device is present
	uint8_t whoamI;
	lsm6dsl_device_id_get(&spec, &whoamI);
	if (whoamI != LSM6DSL_ID)
	{
		LOG_INF("whoamI not matching\n");
		while (1)
			;
	}
	// initialize the the LSM6DSL device
	LOG_INF("whoamI matching\n");
	int sucessed = imu_init(&spec);
	if (sucessed != 0)
	{
		LOG_INF("imu_init failed\n");
		while (1)
			;
	}
	LOG_INF("Main algorithm\n");
	// enter the main algorithm loop
	imu_main_step_algorithm(current_conn);
}
