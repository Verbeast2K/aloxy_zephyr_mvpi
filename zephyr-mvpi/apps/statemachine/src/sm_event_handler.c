#include "inc/sm_event_handler.h"
#include "inc/state_machine.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(event_handler, CONFIG_EVENT_HANDLER_LOG_LEVEL);

#define button1 DT_NODELABEL(button1)
#define button2 DT_NODELABEL(button2)

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(button1, gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(button2, gpios);
static struct gpio_callback button_cb;

/* holds the outer events which should be posted*/
static struct k_event *sm_events;

/* events for internal use */
#define EVENT_BTN_LEFT_DOWN BIT(0)
#define EVENT_BTN_LEFT_UP BIT(1)
#define EVENT_BTN_RIGHT_DOWN BIT(2)
#define EVENT_BTN_RIGHT_UP BIT(3)
#define EVENT_TIMER_3S BIT(4)
#define EVENT_TIMER_10S BIT(5)

enum sm_event_handler_state
{
    IDLE,
    LEFT_PRESSED,
    RIGHT_PRESSED,
    BOTH_PRESSED,
};

static void run_sm(struct k_work *work);
K_WORK_DEFINE(sm_event_handler_run, &run_sm);

static void idle_entry(void *o);
static void idle_run(void *o);
static void left_pressed_entry(void *o);
static void left_pressed_run(void *o);
static void right_pressed_entry(void *o);
static void right_pressed_run(void *o);
static void both_pressed_entry(void *o);
static void both_pressed_run(void *o);

struct smf_state sm_event_handler_states[] = {
    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, NULL, NULL),
    [LEFT_PRESSED] = SMF_CREATE_STATE(left_pressed_entry, left_pressed_run, NULL, NULL),
    [RIGHT_PRESSED] = SMF_CREATE_STATE(right_pressed_entry, right_pressed_run, NULL, NULL),
    [BOTH_PRESSED] = SMF_CREATE_STATE(both_pressed_entry, both_pressed_run, NULL, NULL),
};

// /* User defined object */
struct smf_event_handler_obj
{
    /* This must be first */
    struct smf_ctx ctx;

    /* Events */
    struct k_event input_events;
    int32_t events;

    /* Other state specific data add here */
    uint8_t hold_time;
} smf_event_handler;

static void hold_timer_cb(struct k_timer *timer);
K_TIMER_DEFINE(hold_timer, hold_timer_cb, NULL);

static void button_cb_f(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (pins & BIT(button1.pin))
    {
        if (gpio_pin_get_dt(&button1))
        {
            k_event_post(&smf_event_handler.input_events, EVENT_BTN_RIGHT_DOWN);
        }
        else
        {
            k_event_post(&smf_event_handler.input_events, EVENT_BTN_RIGHT_UP);
        }
    }
    if (pins & BIT(button2.pin))
    {
        if (gpio_pin_get_dt(&button2))
        {
            k_event_post(&smf_event_handler.input_events, EVENT_BTN_LEFT_DOWN);
        }
        else
        {
            k_event_post(&smf_event_handler.input_events, EVENT_BTN_LEFT_UP);
        }
    }
    k_work_submit(&sm_event_handler_run);
}

static int buttons_init()
{
    int ret;

    // BUTTON 1 SETUP
    if (!gpio_is_ready_dt(&button1))
    {
        LOG_ERR("Error: button device %s is not ready\n",
                button1.port->name);
        return 0;
    }

    ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
    if (ret != 0)
    {
        LOG_ERR("Error %d: failed to configure %s pin %d\n",
                ret, button1.port->name, button1.pin);
        return 0;
    }

    ret = gpio_pin_interrupt_configure_dt(&button1,
                                          GPIO_INT_EDGE_BOTH);
    if (ret != 0)
    {
        LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n",
                ret, button1.port->name, button1.pin);
        return 0;
    }

    // BUTTON 2 SETUP
    if (!gpio_is_ready_dt(&button1))
    {
        LOG_ERR("Error: button device %s is not ready\n",
                button1.port->name);
        return 0;
    }

    ret = gpio_pin_configure_dt(&button2, GPIO_INPUT);
    if (ret != 0)
    {
        LOG_ERR("Error %d: failed to configure %s pin %d\n",
                ret, button2.port->name, button2.pin);
        return 0;
    }

    ret = gpio_pin_interrupt_configure_dt(&button2,
                                          GPIO_INT_EDGE_BOTH);
    if (ret != 0)
    {
        LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n",
                ret, button2.port->name, button2.pin);
        return 0;
    }

    /* Add callbacks */
    gpio_init_callback(&button_cb, &button_cb_f, BIT(button1.pin) | BIT(button2.pin));
    gpio_add_callback_dt(&button1, &button_cb);
    gpio_add_callback_dt(&button2, &button_cb);

    return 1;
}

static void idle_entry(void *o)
{
    LOG_DBG("STATE IDLE\n");
    k_timer_stop(&hold_timer);
    smf_event_handler.hold_time = 0;
}

static void idle_run(void *o)
{
    if (smf_event_handler.events & EVENT_BTN_LEFT_DOWN)
    {
        smf_set_state(&smf_event_handler.ctx, &sm_event_handler_states[LEFT_PRESSED]);
    }
    else if (smf_event_handler.events & EVENT_BTN_RIGHT_DOWN)
    {
        smf_set_state(&smf_event_handler.ctx, &sm_event_handler_states[RIGHT_PRESSED]);
    }
}

static void left_pressed_entry(void *o)
{
    LOG_DBG("STATE LEFT\n");
}

static void left_pressed_run(void *o)
{
    if (smf_event_handler.events & EVENT_BTN_LEFT_UP)
    {
        LOG_INF("Left press\n");
        k_event_post(sm_events, EVENT_PRESS_LEFT);
        smf_set_state(&smf_event_handler.ctx, &sm_event_handler_states[IDLE]);
    }
    else if (smf_event_handler.events & EVENT_BTN_RIGHT_DOWN)
    {
        smf_set_state(&smf_event_handler.ctx, &sm_event_handler_states[BOTH_PRESSED]);
    }
}

static void right_pressed_entry(void *o)
{
    LOG_DBG("STATE RIGHT\n");
    smf_event_handler.hold_time = 0;
    k_timer_start(&hold_timer, K_SECONDS(1), K_SECONDS(1));
}

static void right_pressed_run(void *o)
{
    // when pressed the btn right event is a release
    if (smf_event_handler.events & EVENT_BTN_RIGHT_UP)
    {
        LOG_INF("Right press\n");
        k_event_post(sm_events, EVENT_PRESS_RIGHT);
        smf_set_state(&smf_event_handler.ctx, &sm_event_handler_states[IDLE]);
    }
    else if (smf_event_handler.events & EVENT_BTN_LEFT_DOWN)
    {
        smf_set_state(&smf_event_handler.ctx, &sm_event_handler_states[BOTH_PRESSED]);
    }
    else if (smf_event_handler.events & EVENT_TIMER_10S)
    {
        LOG_INF("right hold 10s\n");
        k_event_post(sm_events, EVENT_HOLD_RIGHT_10S);
    }
    else if (smf_event_handler.hold_time)
    {
        LOG_INF("right hold\n");
        k_event_post(sm_events, EVENT_HOLD_RIGHT);
    }
}

static void both_pressed_entry(void *o)
{
    LOG_DBG("STATE BOTH\n");
    smf_event_handler.hold_time = 0;
    k_timer_start(&hold_timer, K_SECONDS(1), K_SECONDS(1));
}

static void both_pressed_run(void *o)
{
    if (smf_event_handler.events & EVENT_TIMER_3S)
    {
        LOG_INF("hold both\n");
        k_event_post(sm_events, EVENT_HOLD_BOTH_3S);
        k_timer_stop(&hold_timer);
    }
    // when pressed the btn left event is a release
    else if (smf_event_handler.events & EVENT_BTN_LEFT_UP)
    {
        LOG_INF("press both\n");
        k_event_post(sm_events, EVENT_PRESS_BOTH);
        smf_set_state(&smf_event_handler.ctx, &sm_event_handler_states[IDLE]);
    }
    // when pressed the btn right event is a release
    else if (smf_event_handler.events & EVENT_BTN_RIGHT_UP)
    {
        LOG_INF("press both\n");
        k_event_post(sm_events, EVENT_PRESS_BOTH);
        smf_set_state(&smf_event_handler.ctx, &sm_event_handler_states[IDLE]);
    }
}

static void run_sm(struct k_work *work)
{
    smf_event_handler.events = k_event_wait(&smf_event_handler.input_events, 0x3F, false, K_NO_WAIT);
    LOG_DBG("Events: %d\n", smf_event_handler.events);
    smf_run_state(SMF_CTX(&smf_event_handler));
    k_event_clear(&smf_event_handler.input_events, 0x3F);
}

static void hold_timer_cb(struct k_timer *timer)
{
    smf_event_handler.hold_time += 1;
    LOG_DBG("Timer: %d\n", smf_event_handler.hold_time);
    if (smf_event_handler.hold_time == 10)
    {
        k_event_post(&smf_event_handler.input_events, EVENT_TIMER_10S);
    }
    else if (smf_event_handler.hold_time == 3)
    {
        k_event_post(&smf_event_handler.input_events, EVENT_TIMER_3S);
    }
    k_work_submit(&sm_event_handler_run);
}

int sm_event_handler_init(struct k_event *events)
{
    buttons_init();

    /* Initialise the outgoing state machine event */
    sm_events = events;
    k_event_init(sm_events);

    /* initialise the input events*/
    k_event_init(&smf_event_handler.input_events);

    /* initialise smf_event_handler state machine*/
    smf_set_initial(SMF_CTX(&smf_event_handler), &sm_event_handler_states[IDLE]);

    return 0;
}
