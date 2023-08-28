/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <sensor_api.h>
#include <zephyr/drivers/gpio.h>

#define INT1_NODE DT_ALIAS(int1)
#define INT2_NODE DT_ALIAS(int2)
#define INT1_GPIO_CONTROLLER DEVICE_DT_GET(DT_GPIO_CTLR(INT1_NODE, gpios))
#define INT1_GPIO_PIN DT_GPIO_PIN(INT1_NODE, gpios)
#define INT1_GPIO_FLAGS (DT_GPIO_FLAGS(INT1_NODE, gpios) | GPIO_INPUT)

#define INT2_GPIO_CONTROLLER DEVICE_DT_GET(DT_GPIO_CTLR(INT2_NODE, gpios))
#define INT2_GPIO_PIN DT_GPIO_PIN(INT2_NODE, gpios)
#define INT2_GPIO_FLAGS (DT_GPIO_FLAGS(INT2_NODE, gpios) | GPIO_INPUT)

/* Declare the GPIO callbacks */
struct gpio_callback int1_cb_data;
struct gpio_callback int2_cb_data;
struct k_work work_int_1;
struct k_work work_int_2;

/* Callbacks for each interrupt */
void int1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_work_submit(&work_int_1);
}

void int2_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_work_submit(&work_int_2);
}

void setup_gpio_interrupt()
{

    k_work_init(&work_int_1, imu_handle_int1);
    k_work_init(&work_int_2, imu_handle_int2);
    /* Ensure the GPIO device is ready */
    if (!device_is_ready(INT1_GPIO_CONTROLLER))
    {
        printk("Error: %s device not ready\n", INT1_GPIO_CONTROLLER->name);
        return;
    }

    if (!device_is_ready(INT2_GPIO_CONTROLLER))
    {
        printk("Error: %s device not ready\n", INT2_GPIO_CONTROLLER->name);
        return;
    }

    /* Configure the GPIOs and initialize the callbacks */
    gpio_pin_configure(INT1_GPIO_CONTROLLER, INT1_GPIO_PIN, INT1_GPIO_FLAGS);
    gpio_pin_interrupt_configure(INT1_GPIO_CONTROLLER, INT1_GPIO_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&int1_cb_data, int1_callback, BIT(INT1_GPIO_PIN));
    gpio_add_callback(INT1_GPIO_CONTROLLER, &int1_cb_data);

    gpio_pin_configure(INT2_GPIO_CONTROLLER, INT2_GPIO_PIN, INT2_GPIO_FLAGS);
    gpio_pin_interrupt_configure(INT2_GPIO_CONTROLLER, INT2_GPIO_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&int2_cb_data, int2_callback, BIT(INT2_GPIO_PIN));
    gpio_add_callback(INT2_GPIO_CONTROLLER, &int2_cb_data);
}