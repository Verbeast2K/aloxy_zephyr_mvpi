#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

#include "inc/serial_interface.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem);

#define UART_BUF_SIZE 0xFF

#define UART_NODE DT_NODELABEL(lpuart1)

#define MODEM_INTERFACE_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(aloxy_modem_interface)

static const struct gpio_dt_spec state_spec = GPIO_DT_SPEC_GET(MODEM_INTERFACE_NODE, state_pin_gpios);
static const struct gpio_dt_spec target_state_spec = GPIO_DT_SPEC_GET(MODEM_INTERFACE_NODE, target_state_pin_gpios);

serial_interface_t *serial_interface;

struct uart_data_t
{
    void *fifo_reserved;
    uint8_t data[UART_BUF_SIZE];
    uint16_t len;
};

uint8_t rx_buff;

uint8_t msg = 0;

bool rx_enabled = false;

// bool nrst_state = false;

// static void alp_handler(serial_interface_t* serial_interface, uint8_t* bytes, uint8_t length)
// {
//     if(length > UART_BUF_SIZE)
//     {
//         LOG_INF("Received data over serial that is too large %i > %i", length, UART_BUF_SIZE);
//         return;
//     }
//     struct uart_data_t* buf = k_malloc(sizeof(*buf));
//     memcpy(buf->data, bytes, length);
//     buf->len = length;
//     LOG_HEXDUMP_INF(buf->data, buf->len, "Received data over serial");
//     k_fifo_put(&fifo_uart_rx_data, buf);
// }

static void ping_handler(serial_interface_t *serial_interface, uint8_t *bytes, uint8_t length)
{
    // LOG_INF("ping handler!");
    if (length > UART_BUF_SIZE)
    {
        LOG_INF("Received data over serial that is too large %i > %i", length, UART_BUF_SIZE);
        return;
    }
    // struct uart_data_t *buf = k_malloc(sizeof(*buf));
    // memcpy(buf->data, bytes, length);
    // buf->len = length;
    LOG_HEXDUMP_INF(bytes, length, "Received data over serial");
    // k_fifo_put(&fifo_uart_rx_data, buf);
}

// #define RST_NODE DT_NODELABEL(nrst)
// static const struct gpio_dt_spec nrst = GPIO_DT_SPEC_GET(RST_NODE, gpios);

#define BT1_NODE DT_NODELABEL(button1)
#if !DT_NODE_HAS_STATUS(BT1_NODE, okay)
#error "Unsupported board: button1 devicetree node is not defined"
#endif

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BT1_NODE, gpios);
static struct gpio_callback button_cb_data;

const struct device *uart = DEVICE_DT_GET(UART_NODE);

// static uint8_t heartbeat_interval_seconds = 5;

void ping_work()
{
    msg += 1;

    int ret;
    // rx_enabled = !rx_enabled;

    // if (rx_enabled)
    // {
    //     ret = uart_rx_enable(uart, &rx_buff, sizeof(rx_buff), SYS_FOREVER_US);
    // }

    // else
    // {
    //     ret = uart_rx_disable(uart);
    // }

    LOG_INF("MSG: %d", msg);
    // serial_interface_transfer_bytes(serial_interface, &msg, 1, SERIAL_MESSAGE_TYPE_PING_REQUEST);
    // serial_interface->rx_buffer_state = BUFFER_STATE_IDLE;

    ret = uart_tx(uart, &msg, sizeof(msg), SYS_FOREVER_US);
    // uart_poll_out(uart, msg);
    // LOG_INF("%d\n", ret);
    // LOG_INF("%d\n", ret);

    // serial_interface_register_handler(serial_interface, NULL, SERIAL_MESSAGE_TYPE_PING_RESPONSE);
}
K_WORK_DEFINE(ping, ping_work);

static void heartbeat_cb()
{
    // LOG_INF("%d\n", msg);
    k_work_submit(&ping);
}

K_TIMER_DEFINE(heartbeat_timer, heartbeat_cb, NULL);

void button_pressed(const struct device *dev, struct gpio_callback *cb,
                    uint32_t pins)
{
    // printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
    // heartbeat_interval_seconds += 1;
    // printk("%d\n", heartbeat_interval_seconds);
    // k_timer_start(&heartbeat_timer, K_SECONDS(heartbeat_interval_seconds), K_SECONDS(heartbeat_interval_seconds));

    // LOG_INF("button pressed\n");
    k_work_submit(&ping);

    // nrst_state = !nrst_state;
    // gpio_pin_set_dt(&nrst, nrst_state);
    // LOG_INF("%d", nrst_state);
}

uint8_t buff[4] = {0x20, 0x30, 0x40, 0x50};

int main()
{
    int ret;

    // if (!device_is_ready(uart))
    // {
    //     printk("Error: uart device is not ready\n");
    //     return 0;
    // }

    // if (!gpio_is_ready_dt(&nrst))
    // {
    //     printk("Error: button device %s is not ready\n",
    //            nrst.port->name);
    //     return 0;
    // }

    // ret = gpio_pin_configure_dt(&nrst, GPIO_OUTPUT);
    // if (ret != 0)
    // {
    //     printk("Error %d: failed to configure %s pin %d\n",
    //            ret, nrst.port->name, nrst.pin);
    //     return 0;
    // }

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

    // gpio_pin_set_dt(&nrst, nrst_state);

    serial_interface = serial_interface_init(uart, &state_spec, &target_state_spec);
    serial_interface_register_handler(serial_interface, ping_handler, SERIAL_MESSAGE_TYPE_PING_RESPONSE);
    msg = 10;
    // serial_interface_transfer_bytes(serial_interface, &msg, 1, SERIAL_MESSAGE_TYPE_PING_REQUEST);

    // k_timer_start(&heartbeat_timer, K_SECONDS(heartbeat_interval_seconds), K_SECONDS(heartbeat_interval_seconds));

    // k_cpu_idle();

    // for (;;)
    // {
    //     /* Wait indefinitely for data to be sent over bluetooth */
    //     struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);
    //     uint16_t max_tx_size = bt_gatt_get_mtu(current_conn) - 3;
    //     for (uint16_t index = 0; index < buf->len;)
    //     {
    //         uint16_t tx_length = buf->len - index;
    //         if (tx_length > max_tx_size)
    //         {
    //             LOG_INF("MTU to small %d < %d", max_tx_size, tx_length);
    //             tx_length = max_tx_size;
    //         }
    //         int error = bt_nus_send(current_conn, &(buf->data[index]), tx_length);
    //         if (error)
    //         {
    //             LOG_WRN("Failed to send data over BLE connection error %d\n", error);
    //             break;
    //         }
    //         index += tx_length;
    //     }
    //     LOG_INF("Send data over BLE");
    //     k_free(buf);
    // }
}