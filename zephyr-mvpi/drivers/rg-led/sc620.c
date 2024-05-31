/*
 * ALOXY CONFIDENTIAL
 * __________________
 *  [2017] - [2024] Aloxy nv
 * All Rights Reserved.
 *  NOTICE: All information contained herein is, and remains
 * the property of aloxy nv and its suppliers,
 * if any. The intellectual and technical concepts contained
 * herein are proprietary to Aloxy nv
 * and its suppliers and may be covered by patents and
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Aloxy nv.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <rg_led/rg_led.h>
#include <errno.h>
// #include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sc620, CONFIG_SC620_LOG_LEVEL);

#define DT_DRV_COMPAT st_sc620

#define SC620_LED_CONTROL 0x00

#define SC620_LED1_DIMMING 0x01 /* LED1 dimming control */
#define SC620_LED2_DIMMING 0x02 /* LED2 dimming control */
#define SC620_LED3_DIMMING 0x03 /* LED3 dimming control */
#define SC620_LED4_DIMMING 0x04 /* LED4 dimming control */
#define SC620_LED5_DIMMING 0x05 /* LED5 dimming control */
#define SC620_LED6_DIMMING 0x06 /* LED6 dimming control */
#define SC620_LED7_DIMMING 0x07 /* LED7 dimming control */
#define SC620_LED8_DIMMING 0x08 /* LED8 dimming control */

struct sc620_config
{
    struct i2c_dt_spec bus;
    uint8_t num_leds;
    const struct gpio_dt_spec gpio_enable;
    uint8_t brightness;
    const struct rg_led_info *leds_info;
};

static const struct rg_led_info *
sc620_led_to_info(
    const struct sc620_config *config, uint32_t led)
{
    if (led < config->num_leds)
    {
        return &config->leds_info[led];
    }

    return NULL;
}

static int sc620_get_info(const struct device *dev, uint32_t led, const struct rg_led_info **info)
{
    const struct sc620_config *config = dev->config;
    const struct rg_led_info *led_info = sc620_led_to_info(config, led);

    if (!led_info)
    {
        return -ENODEV;
    }

    *info = led_info;

    return 0;
}
static void sc620_enable(const struct device *dev, bool enabled)
{
    const struct sc620_config *config = dev->config;

    if (!enabled)
    {
        gpio_pin_set_dt(&config->gpio_enable, false);
    }
    else
    {
        gpio_pin_set_dt(&config->gpio_enable, true);
        i2c_reg_write_byte_dt(&config->bus, SC620_LED1_DIMMING, config->brightness);
        i2c_reg_write_byte_dt(&config->bus, SC620_LED2_DIMMING, config->brightness);
        i2c_reg_write_byte_dt(&config->bus, SC620_LED3_DIMMING, config->brightness);
        i2c_reg_write_byte_dt(&config->bus, SC620_LED4_DIMMING, config->brightness);
        i2c_reg_write_byte_dt(&config->bus, SC620_LED5_DIMMING, config->brightness);
        i2c_reg_write_byte_dt(&config->bus, SC620_LED6_DIMMING, config->brightness);
        i2c_reg_write_byte_dt(&config->bus, SC620_LED7_DIMMING, config->brightness);
        i2c_reg_write_byte_dt(&config->bus, SC620_LED8_DIMMING, config->brightness);
    }
}

static int sc620_on(const struct device *dev, uint32_t led, const enum rg_led_color color)
{
    const struct sc620_config *config = dev->config;
    const struct rg_led_info *led_info = sc620_led_to_info(config, led);

    if (!led_info)
    {
        return -ENODEV;
    }
    sc620_enable(dev, true);

    // read control register, needed for values of other led so we don't overwrite them
    uint8_t buf;
    if (i2c_reg_read_byte_dt(&config->bus, SC620_LED_CONTROL, &buf))
    {
        return -EIO;
    }
    switch (color)
    {
    case RG_LED_COLOR_RED:
        WRITE_BIT(buf, led_info->idx_red, 1);
        WRITE_BIT(buf, led_info->idx_green, 0);
        break;
    case RG_LED_COLOR_GREEN:
        WRITE_BIT(buf, led_info->idx_red, 0);
        WRITE_BIT(buf, led_info->idx_green, 1);
        break;
    case RG_LED_COLOR_YELLOW:
        WRITE_BIT(buf, led_info->idx_red, 1);
        WRITE_BIT(buf, led_info->idx_green, 1);
        break;
    default:
        break;
    }
    return i2c_reg_write_byte_dt(&config->bus, SC620_LED_CONTROL, buf);
}

static int sc620_off(const struct device *dev, uint32_t led)
{
    const struct sc620_config *config = dev->config;
    const struct rg_led_info *led_info = sc620_led_to_info(config, led);

    if (!led_info)
    {
        return -ENODEV;
    }

    // read control register, needed for values of other led so we don't overwrite them
    uint8_t buf;
    if (i2c_reg_read_byte_dt(&config->bus, SC620_LED_CONTROL, &buf))
    {
        return -EIO;
    }
    WRITE_BIT(buf, led_info->idx_red, 0);
    WRITE_BIT(buf, led_info->idx_green, 0);

    if (i2c_reg_write_byte_dt(&config->bus, SC620_LED_CONTROL, buf))
    {
        return -EIO;
    }

    // if all leds are off, set enable to 0 to reduce power consumption
    if (buf == 0x0)
    {
        sc620_enable(dev, false);
    }
    return 0;
}

static int sc620_init(const struct device *dev)
{
    LOG_DBG("sc620_init\n");
    const struct sc620_config *config = dev->config;
    int err;

    if (!i2c_is_ready_dt(&config->bus))
    {
        return -ENODEV;
    }
    LOG_DBG("sc620 i2c is ready\n");

    if (!gpio_is_ready_dt(&config->gpio_enable))
    {
        return -ENODEV;
    }
    LOG_DBG("sc620 gpio is ready\n");
    err = gpio_pin_configure_dt(&config->gpio_enable,
                                GPIO_OUTPUT_INACTIVE);
    LOG_DBG("sc620 gpio configured: %d\n", err);
    if (err < 0)
    {
        return err;
    }
    return 0;
}

static const struct rg_led_driver_api sc620_led_api = {
    .on = sc620_on,
    .off = sc620_off,
    .get_info = sc620_get_info,
};

#define RG_LED_INFO(led_node_id)                      \
    {                                                 \
        .idx_green = DT_PROP(led_node_id, idx_green), \
        .idx_red = DT_PROP(led_node_id, idx_red),     \
    },

#define SC620_DEFINE(id)                                                                          \
    static const struct rg_led_info sc620_leds_##id[] = {DT_INST_FOREACH_CHILD(id, RG_LED_INFO)}; \
    static const struct sc620_config sc620_config_##id = {                                        \
        .bus = I2C_DT_SPEC_INST_GET(id),                                                          \
        .num_leds = ARRAY_SIZE(sc620_leds_##id),                                                  \
        .leds_info = sc620_leds_##id,                                                             \
        .gpio_enable =                                                                            \
            GPIO_DT_SPEC_INST_GET(id, enable_gpios),                                              \
        .brightness = DT_INST_PROP(id, brightness),                                               \
    };                                                                                            \
    DEVICE_DT_INST_DEFINE(id, sc620_init, NULL, NULL, &sc620_config_##id, POST_KERNEL, 90, &sc620_led_api);

DT_INST_FOREACH_STATUS_OKAY(SC620_DEFINE)
