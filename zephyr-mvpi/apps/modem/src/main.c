#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

#include "inc/serial_interface.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem);

#define BT1_NODE DT_NODELABEL(button1)
#if !DT_NODE_HAS_STATUS(BT1_NODE, okay)
#error "Unsupported board: button1 devicetree node is not defined"
#endif

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BT1_NODE, gpios);
static struct gpio_callback button_cb_data;

bool state_pin_on = false;

#define UART_BUF_SIZE 0xFF

#define UART_NODE DT_NODELABEL(lpuart1)

#define MODEM_INTERFACE_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(aloxy_modem_interface)

static const struct gpio_dt_spec state_spec = GPIO_DT_SPEC_GET(MODEM_INTERFACE_NODE, state_pin_gpios);
static const struct gpio_dt_spec target_state_spec = GPIO_DT_SPEC_GET(MODEM_INTERFACE_NODE, target_state_pin_gpios);

serial_interface_t *serial_interface;

uint8_t start_command[3] = {0xB4, 0x02, 0x35};
uint8_t stop_command[3] = {0xB4, 0x02, 0x36};

typedef struct
{
    uint8_t version;
    uint8_t counter;
    uint8_t uplinks;
    uint8_t ACKs;
    uint8_t interval_seconds;
} heartbeat_file_t;

heartbeat_file_t heartbeat_file =
    {
        .version = 0,
        .counter = 0,
        .uplinks = 0,
        .ACKs = 0,
        .interval_seconds = 60};

// uint8_t uart_msg = 20;

void btn_work()
{
    //     if (state_pin_on)
    //     {
    //         state_pin_on = false;
    //         gpio_pin_set_dt(&state_spec, 1);
    //     }
    //     else
    //     {
    //         state_pin_on = true;
    //         gpio_pin_set_dt(&state_spec, 0);
    //     }
    heartbeat_file.interval_seconds += 1;
    LOG_INF("interval_seconds: %d", heartbeat_file.interval_seconds);
}
K_WORK_DEFINE(ping, btn_work);

void btn_pressed(const struct device *dev, struct gpio_callback *cb,
                 uint32_t pins)
{
    k_work_submit(&ping);
}

int ret;

struct uart_data_t
{
    // void *fifo_reserved;
    uint8_t data[UART_BUF_SIZE];
    uint16_t len;
};

// static K_FIFO_DEFINE(fifo_uart_rx_data);

void submit_heartbeat_work();
K_TIMER_DEFINE(heartbeat_timer, submit_heartbeat_work, NULL);

void send_heartbeat()
{
    // serial_interface_transfer_bytes(serial_interface, &start_command, sizeof(start_command), SERIAL_MESSAGE_TYPE_ALP_DATA);
    // k_msleep(100);
    serial_interface_transfer_bytes(serial_interface, &heartbeat_file, sizeof(heartbeat_file), SERIAL_MESSAGE_TYPE_ALP_DATA);
    heartbeat_file.counter += 1;
    // k_msleep(100);
    // serial_interface_transfer_bytes(serial_interface, &stop_command, sizeof(stop_command), SERIAL_MESSAGE_TYPE_ALP_DATA);
    LOG_INF("HEARTBEAT_FILE: version:%d count:%d uplinks:%d ACKs:%d interval_seconds:%d",
            heartbeat_file.version, heartbeat_file.counter, heartbeat_file.uplinks, heartbeat_file.ACKs, heartbeat_file.interval_seconds);
    k_timer_start(&heartbeat_timer, K_SECONDS(heartbeat_file.interval_seconds), K_SECONDS(heartbeat_file.interval_seconds));
}
K_WORK_DEFINE(heartbeat_work, send_heartbeat);

void submit_heartbeat_work()
{
    k_work_submit(&heartbeat_work);
}

static void alp_handler(serial_interface_t *serial_interface, uint8_t *bytes, uint8_t length)
{
    if (length > UART_BUF_SIZE)
    {
        LOG_INF("Received data over serial that is too large %i > %i", length, UART_BUF_SIZE);
        return;
    }
    struct uart_data_t *buf = k_malloc(sizeof(*buf));
    memcpy(buf->data, bytes, length);
    buf->len = length;
    LOG_HEXDUMP_INF(buf->data, buf->len, "Received data over serial");
    // k_fifo_put(&fifo_uart_rx_data, buf);
}

int main()
{
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
    // // Button used to toggle MCU2MODEM_INTERRUPT to check how modem reacts to it

    const struct device *uart = DEVICE_DT_GET(UART_NODE);

    // uart_tx(uart, &start_command, sizeof(start_command), SYS_FOREVER_US);

    LOG_INF("IN MAIN");
    serial_interface = serial_interface_init(uart, &state_spec, &target_state_spec);
    serial_interface_register_handler(serial_interface, alp_handler, SERIAL_MESSAGE_TYPE_ALP_DATA);
    serial_interface_transfer_bytes(serial_interface, &start_command, sizeof(start_command), SERIAL_MESSAGE_TYPE_ALP_DATA);
    k_timer_start(&heartbeat_timer, K_SECONDS(heartbeat_file.interval_seconds), K_SECONDS(heartbeat_file.interval_seconds));

    while (1)
    {
        k_msleep(5000);
        // k_msleep(2000);
        LOG_INF("LOOP");
        //     //     LOG_INF("uart_msg: %d", uart_msg);
        // uart_msg += 1;
        //     k_msleep(5000);
        //     // k_msleep(10000);
        // ret = uart_tx(uart, &start_command, sizeof(start_command), SYS_FOREVER_US);

        // serial_interface_transfer_bytes(serial_interface, &uart_msg, sizeof(uart_msg), SERIAL_MESSAGE_TYPE_ALP_DATA);
        // k_msleep(10000);
        // serial_interface_transfer_bytes(serial_interface, &start_command, sizeof(start_command), SERIAL_MESSAGE_TYPE_ALP_DATA);
        // k_msleep(1000);
        // serial_interface_transfer_bytes(serial_interface, &stop_command, sizeof(stop_command), SERIAL_MESSAGE_TYPE_ALP_DATA);
    }

    return 0;
}