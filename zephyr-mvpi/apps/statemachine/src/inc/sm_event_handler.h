#ifndef ZEPHYR_INCLUDE_SM_EVENT_HANDLER_H
#define ZEPHYR_INCLUDE_SM_EVENT_HANDLER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/smf.h>

#define EVENT_PRESS_LEFT BIT(0)
#define EVENT_PRESS_RIGHT BIT(1)
#define EVENT_PRESS_BOTH BIT(2)
#define EVENT_HOLD_BOTH_3S BIT(3)
#define EVENT_HOLD_RIGHT BIT(4)
#define EVENT_HOLD_RIGHT_10S BIT(5)

/**
 * @def input_event_handler_init()
 * @brief Function to initialise s_obj with events
 */
int sm_event_handler_init(struct k_event *smf_event);

#endif // ZEPHYR_INCLUDE_SM_EVENT_HANDLER_H