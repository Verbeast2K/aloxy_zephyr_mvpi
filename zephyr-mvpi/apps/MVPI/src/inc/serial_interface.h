#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdint.h>

typedef enum
{
    SERIAL_MESSAGE_TYPE_ALP_DATA = 0X01,
    SERIAL_MESSAGE_TYPE_PING_REQUEST = 0X02,
    SERIAL_MESSAGE_TYPE_PING_RESPONSE = 0X03,
    SERIAL_MESSAGE_TYPE_LOGGING = 0X04,
    SERIAL_MESSAGE_TYPE_REBOOTED = 0X05,
} serial_message_type_t;

typedef struct serial_interface serial_interface_t;

typedef void (*cmd_handler_t)(serial_interface_t *serial_interface, uint8_t *bytes, uint8_t length);

serial_interface_t *serial_interface_init(const struct device *uart, const struct gpio_dt_spec *uart_state_pin,
                                          const struct gpio_dt_spec *target_uart_state_pin);

void serial_interface_transfer_bytes(serial_interface_t *serial_interface, const uint8_t *bytes, uint8_t length, serial_message_type_t type);

void serial_interface_register_handler(serial_interface_t *serial_interface, cmd_handler_t cmd_handler, serial_message_type_t type);

void serial_interface_unregister_handler(serial_interface_t *serial_interface, serial_message_type_t type);

void serial_interface_clear_handler(serial_interface_t *serial_interface);

#endif // SERIAL_INTERFACE_H