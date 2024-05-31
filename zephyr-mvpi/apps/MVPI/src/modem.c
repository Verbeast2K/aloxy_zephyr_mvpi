#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

#include "inc/serial_interface.h"
#include "inc/modem.h"
#include "inc/heartbeat.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem, CONFIG_MODEM_LOG_LEVEL);

#define UART_BUF_SIZE 0xFF

#define START_MODEM_TAG_ID 1
#define STOP_MODEM_TAG_ID 2
#define HEARTBEAT_TAG_ID 3

#define ALP_INDIRECT_FORWARD_SPEC_FILE_LORAWAN 0x44
#define ALP_INDIRECT_FORWARD 0x33
#define ALP_REQUEST_TAG 0xB4
#define ALP_RETURNFILE_DATA 0x20
#define OFFSET 0x00

#define MODEM_RESPONSE_LORAWAN_JOINING 0x63
#define MODEM_RESPONSE_LORAWAN_JOINED 0xE3
#define MODEM_RESPONSE_ACK 0xA3

#define HEARTBEAT_INTERVAL_SECONDS 60

#define UART_NODE DT_NODELABEL(lpuart1)

#define MODEM_INTERFACE_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(aloxy_modem_interface)

static const struct gpio_dt_spec state_spec = GPIO_DT_SPEC_GET(MODEM_INTERFACE_NODE, state_pin_gpios);
static const struct gpio_dt_spec target_state_spec = GPIO_DT_SPEC_GET(MODEM_INTERFACE_NODE, target_state_pin_gpios);

const struct device *uart = DEVICE_DT_GET(UART_NODE);
serial_interface_t *serial_interface;
heartbeat_file_t *heartbeat;
heartbeat_payload_t heartbeat_payload;

uint8_t start_command[3] = {0xB4, 0x02, 0x35}; // 180253
uint8_t stop_command[3] = {0xB4, 0x02, 0x36};  // 180254

struct uart_data_t
{
    uint8_t data[UART_BUF_SIZE];
    uint16_t len;
};

void heartbeat_timer_cb();
K_TIMER_DEFINE(heartbeat_timer, heartbeat_timer_cb, NULL);

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

    switch (buf->data[0])
    {
    case MODEM_RESPONSE_ACK:
        switch (buf->data[1])
        {
        case HEARTBEAT_TAG_ID:
            heartbeat_ACK_received(true);
            LOG_INF("Heartbeat transmit success");
            break;

        case (/*START_MODEM_TAG_ID || */ STOP_MODEM_TAG_ID):
            LOG_INF("Modem start/stop success");
            break;

        default:
            break;
        }
        break;

    case MODEM_RESPONSE_LORAWAN_JOINING:
        LOG_INF("JOINING LORAWAN...");
        break;

    case MODEM_RESPONSE_LORAWAN_JOINED:
        LOG_INF("LORAWAN JOINED!");
        break;

    default:
        break;
    }
}

heartbeat_payload_t heartbeat_payload_initialize(heartbeat_file_t *heartbeat_file)
{
    heartbeat_file_t heartbeat_temp = *heartbeat_file;
    heartbeat_payload.request_tag[0] = ALP_REQUEST_TAG;
    heartbeat_payload.request_tag[1] = HEARTBEAT_TAG_ID;
    heartbeat_payload.indirect_forward[0] = ALP_INDIRECT_FORWARD;
    heartbeat_payload.indirect_forward[1] = ALP_INDIRECT_FORWARD_SPEC_FILE_LORAWAN;
    heartbeat_payload.header[0] = ALP_RETURNFILE_DATA;
    heartbeat_payload.header[1] = HEARTBEAT_TAG_ID;
    heartbeat_payload.header[2] = OFFSET;
    heartbeat_payload.header[3] = sizeof(heartbeat_temp);

    heartbeat_payload.heartbeat_file = heartbeat_temp;

    return heartbeat_payload;
}

void serial_interface_initialize()
{
    heartbeat = heartbeat_initialize();
    heartbeat_payload = heartbeat_payload_initialize(heartbeat);
    serial_interface = serial_interface_init(uart, &state_spec, &target_state_spec);
    serial_interface_register_handler(serial_interface, alp_handler, SERIAL_MESSAGE_TYPE_ALP_DATA);
}

void heartbeat_transmit()
{
    heartbeat_uplinks_increment();
    heartbeat_counter_increment();
    heartbeat_payload = heartbeat_payload_initialize(heartbeat);
    serial_interface_transfer_bytes(serial_interface, &heartbeat_payload, sizeof(heartbeat_payload), SERIAL_MESSAGE_TYPE_ALP_DATA);
    k_timer_start(&heartbeat_timer, K_SECONDS(HEARTBEAT_INTERVAL_SECONDS), K_SECONDS(HEARTBEAT_INTERVAL_SECONDS));
    heartbeat_ACK_received(false);
}

K_WORK_DEFINE(heartbeat_transmit_work, heartbeat_transmit);

void heartbeat_timer_cb()
{
    k_work_submit(&heartbeat_transmit_work);
}

void serial_interface_start()
{
    k_timer_start(&heartbeat_timer, K_SECONDS(5), K_SECONDS(HEARTBEAT_INTERVAL_SECONDS));
    serial_interface_transfer_bytes(serial_interface, &start_command, sizeof(start_command), SERIAL_MESSAGE_TYPE_ALP_DATA);
}

void serial_interface_stop()
{
    k_timer_stop(&heartbeat_timer);
    serial_interface_transfer_bytes(serial_interface, &stop_command, sizeof(stop_command), SERIAL_MESSAGE_TYPE_ALP_DATA);
}