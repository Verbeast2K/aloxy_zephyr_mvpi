#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

#include "inc/heartbeat_file.h"

#define ALP_OPERATION_INDIRECT_FORWARD 51
#define ALP_OPERATION_REQUEST_TAG 52
#define ALP_OPERATION_RETURN_FILE_DATA 32

typedef struct
{
    uint8_t counter;
    uint8_t state;
    uint32_t interval_s;
} heartbeat_file_t;

heartbeat_file_t heartbeat_file =
    {
        .counter = 0,
        .state = 0,
        .interval_s = 5};

typedef struct
{
    uint8_t command;
    uint8_t command_id;
} heartbeat_command_t;

typedef struct return_file_data_t
{
    uint8_t command;
    uint8_t file_id;
    uint8_t offset;
    uint8_t length;
    heartbeat_file_t heartbeat_data;
} return_file_data_t;

struct uart_heartbeat
{
    heartbeat_command_t heartbeat_command;
    return_file_data_t return_file_data;
}

// #define HEARTBEAT_FILE_SIZE sizeof(heartbeat_file_t)

enum uart_event_handler_state {
    IDLE,
    TX_BUSY,
    RX_PROCESSING,
};

static void run_sm(struct k_work *work);
K_WORK_DEFINE(uart_event_handler_run, &run_sm);

static void idle_entry(void *o);
static void idle_run(void *o);
static void tx_busy_entry(void *o);
static void tx_busy_run(void *o);
static void rx_processing_entry(void *o);
static void rx_processing_run(void *o);

struct smf_state uart_event_handler_states[] = {
    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, NULL, NULL),
    [TX_BUSY] = SMF_CREATE_STATE(tx_busy_entry, tx_busy_run, NULL, NULL),
    [RX_PROCESSING] = SMF_CREATE_STATE(right_pressed_entry, right_pressed_run, NULL, NULL),
};

// /* User defined object */
struct smf_event_handler_obj
{
    /* This must be first */
    struct smf_ctx ctx;

    /* Events */
    struct k_event input_events;
    int32_t events;

    // /* Other state specific data add here */
    // uint8_t hold_time;
} uart_event_handler;

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(heartbeat);

#define UART_NODE DT_NODELABEL(lpuart1)

#define BT1_NODE DT_NODELABEL(button1)
#if !DT_NODE_HAS_STATUS(BT1_NODE, okay)
#error "Unsupported board: button1 devicetree node is not defined"
#endif

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BT1_NODE, gpios);
static struct gpio_callback button_cb_data;

const struct device *uart = DEVICE_DT_GET(UART_NODE);

heartbeat_command_t request_tag =
    {
        .command = ALP_OPERATION_REQUEST_TAG,
        .command_id = 20};

heartbeat_command_t indirect_forward =
    {
        .command = ALP_OPERATION_INDIRECT_FORWARD,
        .command_id = 68};

return_file_data_t heartbeat_data =
    {
        .command = ALP_OPERATION_RETURN_FILE_DATA,
        .file_id = 90,
        .offset = 0,
        .length = sizeof(heartbeat_file),
        .heartbeat_data = heartbeat_file};

void heartbeat_file_send()
{
    int ret = uart_tx(uart, &heartbeat_file, HEARTBEAT_FILE_SIZE, SYS_FOREVER_US);
    LOG_INF("COUNTER: %d\n", heartbeat_file.counter);
    LOG_INF("STATE: %d\n", heartbeat_file.state);
    LOG_INF("INTERVAL: %d\n", heartbeat_file.interval_s);
    LOG_INF("uart_tx_ret: %d\n", ret);
    heartbeat_file.counter += 1;
}
K_WORK_DEFINE(heartbeat_work, heartbeat_file_send);

void heartbeat_work_submit()
{
    k_work_submit(&heartbeat_work);
}
K_TIMER_DEFINE(heartbeat_timer, heartbeat_work_submit, NULL);

void ping_work()
{
    heartbeat_file.interval_s += 1;
}
K_WORK_DEFINE(ping, ping_work);

void button_pressed(const struct device *dev, struct gpio_callback *cb,
                    uint32_t pins)
{
    // LOG_INF("button pressed\n");
    k_work_submit(&ping);
    k_timer_start(&heartbeat_timer, K_SECONDS(heartbeat_file.interval_s), K_SECONDS(heartbeat_file.interval_s));
}

int main()
{
    int ret;

    if (!device_is_ready(uart))
    {
        printk("Error: uart device is not ready\n");
        return 0;
    }

    if (!gpio_is_ready_dt(&button1))
    {
        printk("Error: button device %s is not ready\n",
               button1.port->name);
        return 0;
    }

    ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
    if (ret != 0)
    {
        printk("Error %d: failed to configure %s pin %d\n",
               ret, button1.port->name, button1.pin);
        return 0;
    }

    ret = gpio_pin_interrupt_configure_dt(&button1,
                                          GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0)
    {
        printk("Error %d: failed to configure interrupt on %s pin %d\n",
               ret, button1.port->name, button1.pin);
        return 0;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button1.pin));
    gpio_add_callback(button1.port, &button_cb_data);
    LOG_INF("Set up button at %s pin %d\n", button1.port->name, button1.pin);

    k_timer_start(&heartbeat_timer, K_SECONDS(heartbeat_file.interval_s), K_SECONDS(heartbeat_file.interval_s));
}