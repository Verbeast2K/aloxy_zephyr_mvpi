/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

#define SLEEP_TIME_MS	1
#define BT1_NODE	DT_NODELABEL(button1)
#define BT2_NODE	DT_NODELABEL(button2)
#if !DT_NODE_HAS_STATUS(BT1_NODE, okay)
#error "Unsupported board: button1 devicetree node is not defined"
#endif
#if !DT_NODE_HAS_STATUS(BT2_NODE, okay)
#error "Unsupported board: button2 devicetree node is not defined"
#endif
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BT1_NODE, gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(BT2_NODE, gpios);
static struct gpio_callback button_cb_data1;
static struct gpio_callback button_cb_data2;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

int main(void)
{
	printk("booted\n");
	int ret;

	if (!gpio_is_ready_dt(&button1)||!gpio_is_ready_dt(&button2)) {
		printk("Error: button device %s is not ready\n",
		       button1.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button1.port->name, button1.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button1,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button1.port->name, button1.pin);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button2, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button2.port->name, button2.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button2,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button2.port->name, button2.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data1, button_pressed, BIT(button1.pin));
	gpio_add_callback(button1.port, &button_cb_data1);
	printk("Set up button at %s pin %d\n", button1.port->name, button1.pin);

	gpio_init_callback(&button_cb_data2, button_pressed, BIT(button2.pin));
	gpio_add_callback(button2.port, &button_cb_data2);
	printk("Set up button at %s pin %d\n", button2.port->name, button2.pin);

	printk("Press a button\n");
	return 0;
}
