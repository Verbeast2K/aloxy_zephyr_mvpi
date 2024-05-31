#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "rg_led/rg_led.h"

const struct device *const led_dev = DEVICE_DT_GET(DT_NODELABEL(ledcontroller));

int main(void)
{
    printk("In main\n");
    if (!led_dev)
    {
        printk("No device found\n");
    }
    else if (!device_is_ready(led_dev))
    {
        printk("led device is not ready\n");
    }
    else
    {
        printk("found led device %s\n", led_dev->name);
    }
    while (1)
    {
        rg_led_on(led_dev, 0, RG_LED_COLOR_GREEN);
        printk("turn led on\n");
        k_msleep(2000);
        rg_led_on(led_dev, 1, RG_LED_COLOR_YELLOW);
        rg_led_off(led_dev, 0);
        printk("turn led off\n");
        k_msleep(2000);
        rg_led_off(led_dev, 1);
    }
}