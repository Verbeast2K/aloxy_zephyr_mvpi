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

#ifndef ZEPHYR_INCLUDE_DRIVERS_RG_LED_H
#define ZEPHYR_INCLUDE_DRIVERS_RG_LED_H

#include <errno.h>

#include <zephyr/types.h>
#include <zephyr/device.h>

// #ifdef __cplusplus
// extern "C"
// {
// #endif

enum rg_led_color
{
    RG_LED_COLOR_RED,
    RG_LED_COLOR_GREEN,
    RG_LED_COLOR_YELLOW
};

/**
 * @brief Info struct for a single RG LED
 *
 * @param idx_red index of the red lED on the controller
 * @param idx_green index of the green LED on the controller
 *
 */
struct rg_led_info
{
    uint32_t idx_red;
    uint32_t idx_green;
};

/**
 * @typedef rg_led_api_on()
 * @brief Callback API for turning on an LED
 *
 * @see rg_led_on() for argument descriptions.
 */
typedef int (*rg_led_api_on)(const struct device *dev, uint32_t led, const enum rg_led_color color);

/**
 * @typedef rg_led_api_off()
 * @brief Callback API for turning off an LED
 *
 * @see rg_led_off() for argument descriptions.
 */
typedef int (*rg_led_api_off)(const struct device *dev, uint32_t led);

/**
 * @typedef led_api_get_info()
 * @brief API callback to get LED information
 *
 * @see rg_led_get_info() for argument descriptions.
 */
typedef int (*rg_led_api_get_info)(const struct device *dev, uint32_t led,
                                   const struct rg_led_info **info);

struct rg_led_driver_api
{
    rg_led_api_on on;
    rg_led_api_off off;
    rg_led_api_get_info get_info;
};

/**
 * @brief Get RG LED information
 *
 * This routine provides information about an RG LED.
 *
 * @param dev RG LED device
 * @param led LED number
 * @param info Pointer to a pointer filled with RG LED information
 * @return 0 on success, negative on error
 */
static inline int rg_led_get_info(const struct device *dev, uint32_t led,
                                  const struct rg_led_info **info)
{
    const struct rg_led_driver_api *api =
        (const struct rg_led_driver_api *)dev->api;

    return api->get_info(dev, led, info);
}

/**
 * @brief Turn on an RG LED
 *
 * This routine turns on an RG LED
 *
 * @param dev RG LED device
 * @param led LED number
 * @param color Color led should be
 * @return 0 on success, negative on error
 */
static inline int rg_led_on(const struct device *dev, uint32_t led, const enum rg_led_color color)
{
    const struct rg_led_driver_api *api =
        (const struct rg_led_driver_api *)dev->api;

    return api->on(dev, led, color);
}

/**
 * @brief Turn off an RG LED
 *
 * This routine turns on an RG LED
 *
 * @param dev RG LED device
 * @param led LED number
 * @return 0 on success, negative on error
 */
static inline int rg_led_off(const struct device *dev, uint32_t led)
{
    const struct rg_led_driver_api *api =
        (const struct rg_led_driver_api *)dev->api;

    return api->off(dev, led);
}

// #ifdef __cplusplus
// }
// #endif

#endif