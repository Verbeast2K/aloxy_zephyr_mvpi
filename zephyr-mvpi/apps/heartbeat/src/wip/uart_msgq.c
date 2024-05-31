#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/smf.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem);

#define UART_NODE DT_NODELABEL(lpuart1)

#define BT1_NODE DT_NODELABEL(button1)
#if !DT_NODE_HAS_STATUS(BT1_NODE, okay)
#error "Unsupported board: button1 devicetree node is not defined"
#endif

#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256

#define RX_TIMEOUT_USEC 10000
#define MODEM_TIMEOUT_MSEC 10000

// K_FIFO_DEFINE(uart_fifo);
K_MSGQ_DEFINE(uart_queue, 1, UART_TX_BUF_SIZE, 1);

int ret;
int queue_msg;
bool TX_DONE = true;

enum uart_event_handler_state
{
    IDLE,
    TX_BUSY,
    RX_PROCESSING,
};

static void run_sm();
K_WORK_DEFINE(uart_event_handler_run, &run_sm);

static void modem_worker();
K_WORK_DEFINE(modem_work, modem_worker);

static void modem_timeout_callback(struct k_timer *timer);
K_TIMER_DEFINE(modem_timeout, modem_timeout_callback, NULL);

static void idle_entry(void *o);
static void idle_run(void *o);
static void tx_busy_entry(void *o);
static void tx_busy_run(void *o);
static void rx_processing_entry(void *o);
static void rx_processing_run(void *o);

static int transmit(const struct device *dev, const uint8_t *buf, size_t len, int32_t timeout)
{
    while (!TX_DONE)
    {
    }
    TX_DONE = false;
    uart_tx(dev, buf, len, timeout);
}

struct smf_state uart_event_handler_states[] = {
    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, NULL, NULL),
    [TX_BUSY] = SMF_CREATE_STATE(tx_busy_entry, tx_busy_run, NULL, NULL),
    [RX_PROCESSING] = SMF_CREATE_STATE(rx_processing_entry, rx_processing_run, NULL, NULL),
};

// /* User defined object */
struct smf_event_handler_obj
{
    /* This must be first */
    struct smf_ctx ctx;

    /* Events */
    struct k_event uart_events;
    int32_t events;

    // /* Other state specific data add here */
    uint8_t data;
    // bool modem_state;
} uart_event_handler;

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BT1_NODE, gpios);
static struct gpio_callback button_cb_data;

uint8_t start_command[3] = {0xB4, 0x02, 0x35};
uint8_t stop_command[3] = {0xB4, 0x02, 0x36};

uint8_t msg = 2;

uint8_t rx_buffer[UART_RX_BUF_SIZE];

const struct device *uart = DEVICE_DT_GET(UART_NODE);

static void start_modem(bool on)
{
    // uart_event_handler.modem_state = on;

    // if (on)
    // {
    //     k_msgq_put(&uart_queue, &start_command, K_NO_WAIT);
    // }
    // else if (!on)
    // {
    //     k_msgq_put(&uart_queue, &stop_command, K_NO_WAIT);
    // }
    if (on)
    {
        ret = uart_tx(uart, &start_command, sizeof(start_command), SYS_FOREVER_US);
        LOG_INF("start_modem_communication: %d%d%d\n", start_command[0], start_command[1], start_command[2]);
    }
    else if (!on)
    {
        ret = uart_tx(uart, &stop_command, sizeof(stop_command), SYS_FOREVER_US);
        LOG_INF("stop_modem_communication: %d%d%d\n", stop_command[0], stop_command[1], stop_command[2]);
    }
    k_msleep(10);
}

void modem_worker()
{
    start_modem(false);
    // uart_event_handler.modem_state = false;
}
static void modem_timeout_callback(struct k_timer *timer)
{
    k_work_submit(&modem_work);
}

void btn_work()
{
    LOG_INF("BTN_WORK\n");

    transmit(uart, &stop_command, sizeof(stop_command), SYS_FOREVER_US);

    // transfer_data(&start_command);
    // uart_tx(uart, &stop_command, sizeof(stop_command), SYS_FOREVER_US);

    // msg += 1;
    // LOG_INF("MSG: %d", msg);
    // queue_msg = msg;
    // k_fifo_put(&uart_fifo, &fifo_msg);
    // k_msgq_put(&uart_queue, &queue_msg, K_NO_WAIT);
    // ret = k_fifo_get(&uart_fifo, K_FOREVER);
    // k_msgq_get(&uart_queue, &ret, K_FOREVER);
    // LOG_INF("queue_msg: %d", ret);

    // k_fifo_put(&uart_fifo, &start_command);
    // transfer_data(&rx_buffer);

    // ret = uart_tx(uart, &start_command, sizeof(start_command), SYS_FOREVER_US);
    // start_modem(true);
    // k_usleep(1);
    // ret = uart_tx(uart, &rx_buffer, sizeof(rx_buffer), SYS_FOREVER_US);
    // ret = uart_tx_abort(uart);
    // LOG_INF("RET_TX: %d\n", ret);
    // k_usleep(1);
    // start_modem(false);
    // LOG_INF("start_command: %d%d%d", start_command[0], start_command[1], start_command[2]);

    // ret = uart_rx_disable(uart);

    // ret = uart_tx(uart, &stop_command, sizeof(stop_command), SYS_FOREVER_US);

    // ret = uart_tx(uart, &msg, sizeof(msg), SYS_FOREVER_US);

    // LOG_INF("stop command: %d%d%d\n", stop_command[0], stop_command[1], stop_command[2]);
    // LOG_INF("%d\n", msg);
}
K_WORK_DEFINE(ping, btn_work);

void btn_pressed(const struct device *dev, struct gpio_callback *cb,
                 uint32_t pins)
{
    k_work_submit(&ping);
}

void idle_entry(void *o)
{
    LOG_INF("IDLE_ENTRY");
}

void idle_run(void *o)
{
    // k_msgq_get(&uart_queue, &ret, K_NO_WAIT);
    // if (ret)
    // {
    //     // if (uart_event_handler.modem_state == false)
    //     // {
    //     //     start_modem(true);
    //     // }
    //     LOG_INF("RET: %d", ret);
    //     k_msgq_get(&uart_queue, &uart_event_handler.data, K_NO_WAIT);
    //     smf_set_state(&uart_event_handler.ctx, &uart_event_handler_states[TX_BUSY]);
    // }
    // if (!k_fifo_is_empty(&uart_fifo))
    // {
    //     if (uart_event_handler.modem_state == false)
    //     {
    //         start_modem(true);
    //     }
    //     LOG_INF("RET: %d", ret);
    //     uart_event_handler.data = k_fifo_get(&uart_fifo, K_FOREVER);
    //     smf_set_state(&uart_event_handler.ctx, &uart_event_handler_states[TX_BUSY]);
    // }

    // LOG_INF("IDLE_RUN");
}

void tx_busy_entry(void *o)
{
    LOG_INF("TX_BUSY_ENTRY");
}

void tx_busy_run(void *o)
{
    // k_timer_start(&modem_timeout, K_MSEC(MODEM_TIMEOUT_MSEC), K_NO_WAIT);
    LOG_INF("TX_BUSY_RUN");
    // if (k_fifo_is_empty(&uart_fifo))
    // {
    //     smf_set_state
    // }
    // uart_event_handler.data = k_fifo_get(&uart_fifo, K_FOREVER);
    k_msgq_get(&uart_queue, &uart_event_handler.data, K_FOREVER);
    LOG_INF("TX_DATA: %d", uart_event_handler.data);
    ret = uart_tx(uart, &uart_event_handler.data, sizeof(&uart_event_handler.data), SYS_FOREVER_US);
}

void rx_processing_entry(void *o)
{
    LOG_INF("RX_PROCESSING_ENTRY");
}

void rx_processing_run(void *o)
{
}

static void run_sm()
{
    while (1)
    {
        k_msleep(100);
        ret = smf_run_state(SMF_CTX(&uart_event_handler));
    }
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    LOG_DBG("evt->type %d", evt->type);
    switch (evt->type)
    {
    case UART_TX_DONE:
        LOG_INF("UART_TX_DONE");
        TX_DONE = true;
        // smf_set_state(&uart_event_handler.ctx, &uart_event_handler_states[IDLE]);
    case UART_TX_ABORTED:
    case UART_RX_RDY:
        // smf_set_state(&uart_event_handler.ctx, &uart_event_handler_states[RX_PROCESSING]);
        // TO ADD: HANDLE RX
        LOG_INF("UART_RX_RDY");
    case UART_RX_BUF_REQUEST:
    case UART_RX_BUF_RELEASED:
    case UART_RX_DISABLED:
        break;
    case UART_RX_STOPPED:
        break;
    }
}

static int modem_init(const struct device *dev)
{
    if (!device_is_ready(dev))
    {
        printk("Error: uart device is not ready\n");
        return 0;
    }
    ret = uart_callback_set(dev, uart_cb, dev->data);
    return 1;
}

// void transfer_data(uint8_t *bytes)
// {
//     uart_event_handler.data = bytes;
//     smf_set_state(&uart_event_handler.ctx, &uart_event_handler_states[TX_BUSY]);
// }

int main()
{
    modem_init(uart);
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

    gpio_init_callback(&button_cb_data, btn_pressed, BIT(button1.pin));
    gpio_add_callback(button1.port, &button_cb_data);
    LOG_INF("Set up button at %s pin %d\n", button1.port->name, button1.pin);

    ret = uart_rx_disable(uart);
    ret = uart_rx_enable(uart, &rx_buffer, sizeof(rx_buffer), RX_TIMEOUT_USEC);
    smf_set_initial(SMF_CTX(&uart_event_handler), &uart_event_handler_states[IDLE]);

    // int fifo_msg = msg;
    // k_fifo_put(&uart_fifo, &fifo_msg);
    // int fifo_adder = &start_command;
    // k_fifo_put(&uart_fifo, &start_command);
    // fifo_adder = &stop_command;
    // k_fifo_put(&uart_fifo, &stop_command);

    // k_msleep(1000);

    run_sm();
}
