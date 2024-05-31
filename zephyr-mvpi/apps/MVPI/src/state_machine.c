#include "inc/state_machine.h"
#include "inc/sm_event_handler.h"
#include "inc/modem.h"

#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>
#include "rg_led/rg_led.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sm, CONFIG_SM_LOG_LEVEL);

int ret;

const struct device *const led_dev = DEVICE_DT_GET(DT_NODELABEL(ledcontroller));

/* Forward declaration of state table */
struct smf_state mvpi_states[];

// /* User defined object */
struct s_object
{
	/* This must be first */
	struct smf_ctx ctx;

	/* Events */
	struct k_event smf_event;
	int32_t events;

	/* Other state specific data add here */
} s_obj;

/* List of MVPI states */
enum mvpi_state
{
	Standby,
	Operational,
	Configuration,
	Configuration_idle,
	Reset_menu,
	Standby_state,
	Factory_reset
};

uint8_t state_index = Standby;

/* List of rg_led colors including "off" */
enum led_color
{
	LED_COLOR_RED = RG_LED_COLOR_RED,
	LED_COLOR_GREEN = RG_LED_COLOR_GREEN,
	LED_COLOR_YELLOW = RG_LED_COLOR_YELLOW,
	LED_COLOR_OFF = 3,
};

enum led
{
	LED_LEFT = 0,
	LED_RIGHT = 1,
};

struct state_leds
{
	enum led_color led_color_left;
	enum led_color led_color_right;
};

/**
 *  @def leds_off()
 * @brief Function to turn off both LEDs
 */
void leds_off();

/**
 *  @def leds_off_caller()
 * @brief Function to start timer for calling leds_off
 */
void leds_off_caller();

/**
 *  @def show_state()
 * @param stay_on   Flag that will keep the LED state display on if enabled
 * @brief Function to show state using LEDs
 */
void show_state(bool stay_on);

/**
 *  @def timeout_handler()
 * @brief Function to revert to Operational state from Configuration
 */
void timeout_handler();

/**
 *  @def timeout_handler_caller()
 * @brief Function to start timer for calling timeout_handler
 */
void timeout_handler_caller();

/**
 *  @def standby_entry()
 * @brief Function to run once, on entering Standby state
 */
void Standby_entry(void *o);

/**
 *  @def standby_run()
 * @brief Main function to run in Standby state
 */
void Standby_run(void *o);

/**
 *  @def standby_exit()
 * @brief Function to run once, on exiting Standby state
 */
void Standby_exit(void *o);

/**
 *  @def operational_entry()
 * @brief Function to run once, on entering Operational state
 */
void Operational_entry(void *o);

/**
 *  @def operational_run()
 * @brief Main function to run in Operational state
 */
void Operational_run(void *o);

/**
 *  @def operational_exit()
 * @brief Function to run once, on exiting Operational state
 */
void Operational_exit(void *o);

/**
 *  @def configuration_entry()
 * @brief Function to run once, on entering Configuration state
 */
void Configuration_entry(void *o);

/**
 *  @def configuration_exit()
 * @brief Function to run once, on exiting Configuration state
 */
void Configuration_exit(void *o);

/**
 *  @def configuration_idle_entry()
 * @brief Function to run once, on entering Configuration_idle state
 */
void Configuration_idle_entry(void *o);

/**
 *  @def configuration_idle_run()
 * @brief Main function to run in Configuration_idle state
 */
void Configuration_idle_run(void *o);

/**
 *  @def Reset_menu_entry()
 * @brief Function to run once, on entering Reset_menu state
 */
void Reset_menu_entry(void *o);

/**
 *  @def Reset_menu_run()
 * @brief Main function to run in Reset_menu state
 */
void Reset_menu_run(void *o);

void Reset_menu_exit(void *o);

/**
 *  @def Standby_state_entry()
 * @brief Function to run once, on entering Standby_state state
 */
void Standby_state_entry(void *o);

/**
 *  @def Standby_state_run()
 * @brief Main function to run in Standby_state state
 */
void Standby_state_run(void *o);

void Standby_state_exit(void *o);

/**
 *  @def Factory_reset_entry()
 * @brief Function to run once, on entering Factory_reset state
 */
void Factory_reset_entry(void *o);

/**
 *  @def Factory_reset_run()
 * @brief Main function to run in Factory_reset state
 */
void Factory_reset_run(void *o);

void Factory_reset_exit(void *o);

const struct state_leds state_to_state_leds[] = {
	[Standby] = {LED_COLOR_RED, LED_COLOR_RED},
	[Operational] = {LED_COLOR_GREEN, LED_COLOR_GREEN},
	[Configuration] = {LED_COLOR_YELLOW, LED_COLOR_YELLOW},
	[Configuration_idle] = {LED_COLOR_OFF, LED_COLOR_OFF},
	[Reset_menu] = {LED_COLOR_RED, LED_COLOR_OFF},
	[Standby_state] = {LED_COLOR_RED, LED_COLOR_GREEN},
	[Factory_reset] = {LED_COLOR_RED, LED_COLOR_RED}};

// Define function to turn off both leds
void leds_off()
{
	LOG_DBG("leds off\n");
	rg_led_off(led_dev, LED_LEFT);
	rg_led_off(led_dev, LED_RIGHT);
}
K_WORK_DEFINE(leds_off_work, leds_off);

// Define function to submit leds_off to workqueue using the timer
void leds_off_caller()
{
	k_work_submit(&leds_off_work);
}
K_TIMER_DEFINE(led_timer, leds_off_caller, NULL);

void show_state(bool stay_on)
{
	struct state_leds led_colors = state_to_state_leds[state_index];

	if (led_colors.led_color_left == LED_COLOR_OFF)
	{
		rg_led_off(led_dev, LED_LEFT);
	}
	else
	{
		rg_led_on(led_dev, LED_LEFT, led_colors.led_color_left);
	}

	if (led_colors.led_color_right == LED_COLOR_OFF)
	{
		rg_led_off(led_dev, LED_RIGHT);
	}
	else
	{
		rg_led_on(led_dev, LED_RIGHT, led_colors.led_color_right);
	}

	if (!stay_on)
	{
		k_timer_start(&led_timer, K_SECONDS(1), K_NO_WAIT);
	}
}

/* 30S timer for configuration timeout */
void timeout_handler()
{
	if (state_index > Operational)
	{
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Operational]);
	}
}
K_WORK_DEFINE(timeout_handler_work, timeout_handler);

void timeout_handler_caller()
{
	k_work_submit(&timeout_handler_work);
}
K_TIMER_DEFINE(timeout, timeout_handler_caller, NULL);

/* 1S timer for exiting Configuration_idle mode */
void enter_configuration_idle()
{
	leds_off();
}
K_WORK_DEFINE(enter_configuration_idle_work, enter_configuration_idle);

void enter_configuration_idle_caller()
{
	k_work_submit(&enter_configuration_idle_work);
}
K_TIMER_DEFINE(configuration_led_timeout, enter_configuration_idle_caller, NULL);

/* State Standby */
void Standby_entry(void *o)
{
	LOG_INF("Standby\n");
	state_index = Standby;
	show_state(false);
}

void Standby_run(void *o)
{
	ret = k_event_wait(&s_obj.smf_event, 0x1F, false, K_NO_WAIT);
	k_event_clear(&s_obj.smf_event, 0x1F);

	switch (ret)
	{
	case EVENT_HOLD_BOTH_3S:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Operational]);
		break;

	case EVENT_HOLD_RIGHT:
		show_state(true);
		break;

	case EVENT_PRESS_RIGHT:
		leds_off();
		break;

	default:
		break;
	}
}

void Standby_exit(void *o)
{
	leds_off();
}

/* State Operational */
void Operational_entry(void *o)
{
	LOG_INF("Operational\n");
	state_index = Operational;
	show_state(false);
	serial_interface_start();
}

void Operational_run(void *o)
{
	ret = k_event_wait(&s_obj.smf_event, 0x1F, false, K_NO_WAIT);
	k_event_clear(&s_obj.smf_event, 0x1F);

	switch (ret)
	{
	case EVENT_HOLD_BOTH_3S:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Configuration_idle]);
		break;

	case EVENT_HOLD_RIGHT:
		show_state(true);
		break;

	case EVENT_PRESS_RIGHT:
		leds_off();
		break;

		// case EVENT_PRESS_LEFT:
		// 	serial_interface_transfer_bytes(serial_interface, &uart_msg, sizeof(uart_msg), SERIAL_MESSAGE_TYPE_ALP_DATA);
		// 	uart_msg += 1;

	default:
		break;
	}
}

void Operational_exit(void *o)
{
	leds_off();
	serial_interface_stop();
}

void Configuration_entry(void *o)
{
	state_index = Configuration;
	show_state(false);
	k_timer_start(&configuration_led_timeout, K_SECONDS(1), K_NO_WAIT);
}

void Configuration_exit(void *o)
{
	leds_off();
}

void Configuration_idle_entry(void *o)
{
	state_index = Configuration_idle;
	LOG_INF("Configuration_idle\n");
}

void Configuration_idle_run(void *o)
{
	ret = k_event_wait(&s_obj.smf_event, 0x1F, false, K_NO_WAIT);
	k_event_clear(&s_obj.smf_event, 0x1F);

	switch (ret)
	{
	case EVENT_HOLD_BOTH_3S:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Operational]);
		break;

	case EVENT_PRESS_LEFT:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Reset_menu]);
		break;
	}
}

/* State Reset_menu */
void Reset_menu_entry(void *o)
{
	state_index = Reset_menu;
	LOG_INF("Reset_menu\n");
	show_state(true);

	k_timer_start(&timeout, K_SECONDS(30), K_NO_WAIT);
}

void Reset_menu_run(void *o)
{
	ret = k_event_wait(&s_obj.smf_event, 0x1F, false, K_NO_WAIT);
	k_event_clear(&s_obj.smf_event, 0x1F);

	switch (ret)
	{
	case EVENT_HOLD_BOTH_3S:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Operational]);
		break;

	case EVENT_PRESS_LEFT:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Configuration_idle]);
		break;

	case EVENT_PRESS_RIGHT:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Standby_state]);
		break;

	default:
		break;
	}
}

void Reset_menu_exit(void *o)
{
	leds_off();
}

/* State Standby_state */
void Standby_state_entry(void *o)
{
	k_timer_start(&timeout, K_SECONDS(30), K_NO_WAIT);

	LOG_INF("Standby_state\n");
	state_index = Standby_state;
	show_state(true);
}

void Standby_state_run(void *o)
{
	ret = k_event_wait(&s_obj.smf_event, 0x1F, false, K_NO_WAIT);
	k_event_clear(&s_obj.smf_event, 0x1F);

	switch (ret)
	{
	case EVENT_HOLD_BOTH_3S:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Operational]);
		break;

	case EVENT_PRESS_RIGHT:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Factory_reset]);
		break;

	case EVENT_PRESS_BOTH:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Standby]);
		break;

	default:
		break;
	}
}

void Standby_state_exit(void *o)
{
	leds_off();
}

/* State Factory_reset */
void Factory_reset_entry(void *o)
{
	k_timer_start(&timeout, K_SECONDS(30), K_NO_WAIT);

	LOG_INF("Factory_reset\n");
	state_index = Factory_reset;
	show_state(true);
}

void Factory_reset_run(void *o)
{
	ret = k_event_wait(&s_obj.smf_event, 0x1F, false, K_NO_WAIT);
	k_event_clear(&s_obj.smf_event, 0x1F);

	switch (ret)
	{
	case EVENT_HOLD_BOTH_3S:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Operational]);
		break;

	case EVENT_PRESS_RIGHT:
		smf_set_state(SMF_CTX(&s_obj), &mvpi_states[Reset_menu]);
		break;

	case EVENT_PRESS_BOTH:
		printk("Factory Resetting...\n");
		break;

	default:
		break;
	}
}

void Factory_reset_exit(void *o)
{
	leds_off();
}

/* Populate state table */
struct smf_state mvpi_states[] = {
	[Standby] = SMF_CREATE_STATE(Standby_entry, Standby_run, Standby_exit, NULL),
	[Operational] = SMF_CREATE_STATE(Operational_entry, Operational_run, Operational_exit, NULL),
	[Configuration] = SMF_CREATE_STATE(Configuration_entry, NULL, Configuration_exit, NULL),
	[Configuration_idle] = SMF_CREATE_STATE(Configuration_idle_entry, Configuration_idle_run, NULL, &mvpi_states[Configuration]),
	[Reset_menu] = SMF_CREATE_STATE(Reset_menu_entry, Reset_menu_run, Reset_menu_exit, &mvpi_states[Configuration]),
	[Standby_state] = SMF_CREATE_STATE(Standby_state_entry, Standby_state_run, Standby_state_exit, &mvpi_states[Configuration]),
	[Factory_reset] = SMF_CREATE_STATE(Factory_reset_entry, Factory_reset_run, Factory_reset_exit, &mvpi_states[Configuration]),
};

void state_machine_init()
{

	sm_event_handler_init(&s_obj.smf_event);

	/* Setup led controller */
	if (!led_dev)
	{
		LOG_ERR("No device found\n");
	}
	else if (!device_is_ready(led_dev))
	{
		LOG_ERR("led device is not ready\n");
	}
	else
	{
		LOG_INF("found led device %s\n", led_dev->name);
	}

	/* Set initial state */
	smf_set_initial(SMF_CTX(&s_obj), &mvpi_states[Standby]);
}

void state_machine_run()
{
	/* Run the state machine */
	while (1)
	{
		k_msleep(100);
		ret = smf_run_state(SMF_CTX(&s_obj));
	}
}