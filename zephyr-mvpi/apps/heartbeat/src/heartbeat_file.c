#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

#include "inc/heartbeat_file.h"

typedef struct
{
    uint8_t counter;
    uint8_t state;
} heartbeat_file_t;

heartbeat_file_t heartbeat_file =
    {
        .counter = 0,
        .state = 0};

// heartbeat_file_initialize()
// {

// }

// heartbeat_file_send()
// {
//     int ret = uart_tx(heartbeat_file)
// }