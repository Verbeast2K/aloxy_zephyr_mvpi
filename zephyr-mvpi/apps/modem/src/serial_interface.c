#include "serial_interface.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>

// #include "nrfx_gpiote.h"
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/pm.h>

LOG_MODULE_REGISTER(serial_interface, CONFIG_SERIAL_INTERFACE_LOG_LEVEL);

#define SERIAL_INTERFACE_COUNT 1

#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256

#define SERIAL_FRAME_SYNC_BYTE 0xC0
#define SERIAL_FRAME_VERSION 0x00
#define SERIAL_FRAME_HEADER_SIZE 7
#define SERIAL_FRAME_SIZE 4
#define SERIAL_FRAME_COUNTER 2
#define SERIAL_FRAME_TYPE 3
#define SERIAL_FRAME_CRC1 5
#define SERIAL_FRAME_CRC2 6

#define RX_TIMEOUT 100

typedef enum
{
    STATE_IDLE,
    STATE_REQ_START,
    STATE_REQ_WAIT,
    STATE_REQ_BUSY,
    STATE_RESP,
    STATE_RESP_PENDING_REQ,
    STATE_REC
} state_t;

typedef enum
{
    BUFFER_STATE_IDLE,
    BUFFER_STATE_UART,
    BUFFER_STATE_PROCESS,
} buffer_state_t;

#define SWITCH_STATE(state, s)       \
    do                               \
    {                                \
        state = s;                   \
        LOG_DBG("switch to %s", #s); \
    } while (0)

struct serial_interface
{
    const struct gpio_dt_spec *uart_state_pin;        // MCU2MODEM Interrupt pin
    const struct gpio_dt_spec *target_uart_state_pin; // MODEM2MCU Interrupt pin
    struct gpio_callback target_state_callback;       // MODEM2MCU Interrupt callback
    state_t state;                                    // state in "execute_state_machine"
    bool request_pending;                             // Transmitting message
    bool tx_flush_busy;                               //
    const struct device *uart;                        // UART device node
    uint8_t rx_buffer[UART_RX_BUF_SIZE];              //
    uint8_t tx_buffer[UART_TX_BUF_SIZE];              //
    uint8_t tx_fifo_buffer[UART_TX_BUF_SIZE];         //
    buffer_state_t rx_buffer_state;                   //
    bool rx_active;                                   //
    bool tx_active;                                   //
    uint8_t bytes_received;                           //
    uint8_t receive_offset;                           //
    uint32_t bytes_to_transmit;                       //
    bool shutdown;                                    //
    cmd_handler_t alp_handler;                        //
    cmd_handler_t ping_handler;                       //
    cmd_handler_t logging_handler;                    //
    uint8_t packet_up_counter;                        //
    uint8_t packet_down_counter;                      //
    struct k_work process_rx_work;                    //
    struct k_work tx_work;                            //
    struct k_work state_machine_work;                 //
};

static serial_interface_t serial_interfaces[SERIAL_INTERFACE_COUNT];
static uint32_t serial_interface_count;
static void uart_event_handler(const struct device *dev, struct uart_event *evt, void *user_data);
static void execute_state_machine(struct k_work *work);

static void serial_interface_enable(serial_interface_t *serial_interface)
{
    int ret = pm_device_action_run(serial_interface->uart, PM_DEVICE_ACTION_RESUME);
    LOG_INF("PM_RET1: %d", ret);

    serial_interface->shutdown = false;
    serial_interface->rx_active = true;
    serial_interface->rx_buffer_state = BUFFER_STATE_UART;
    ret = uart_rx_enable(serial_interface->uart, serial_interface->rx_buffer, UART_RX_BUF_SIZE, SYS_FOREVER_MS);
    __ASSERT(ret == 0, "uart_rx_enable returned with error %d", ret);
    LOG_DBG("UART enabled");
}

/** @Brief disables UART interface
 *  @return void
 */
static void serial_interface_disable(serial_interface_t *serial_interface)
{
    LOG_DBG("serial_interface_disable %d %d", serial_interface->rx_active, serial_interface->tx_active);
    if (serial_interface->rx_active || serial_interface->tx_active)
    {
        serial_interface->shutdown = true;
        if (serial_interface->rx_active)
        {
            LOG_DBG("uart_rx_disable");
            uart_rx_disable(serial_interface->uart);
        }
        if (serial_interface->tx_active)
        {
            uart_tx_abort(serial_interface->uart);
        }
    }
    else
    {
#if 0
        nrfx_uart_uninit(&serial_interface->uart);
        NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
		NRF_CLOCK->TASKS_HFCLKSTART = 0;
		/* Wait for the external oscillator to start up */
		while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {} 
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
        LOG_DBG("UART disabled");
#endif
        LOG_DBG("UART disabled");
        int ret = pm_device_action_run(serial_interface->uart, PM_DEVICE_ACTION_SUSPEND);
        LOG_INF("PM_RET2: %d", ret);

        gpio_pin_set_dt(serial_interface->uart_state_pin, 0);
    }
}

static void listen(serial_interface_t *serial_interface)
{
    serial_interface_enable(serial_interface);
    gpio_pin_set_dt(serial_interface->uart_state_pin, 1); // set interrupt gpio to indicate ready for data
}

static void release_receiver(serial_interface_t *serial_interface)
{
    LOG_DBG("release_receiver");
    serial_interface_disable(serial_interface);
}

static bool verify_payload(serial_interface_t *serial_interface, uint8_t *bytes, uint8_t length, uint8_t *header)
{
    serial_interface->packet_down_counter++;
    if (header[SERIAL_FRAME_COUNTER] != serial_interface->packet_down_counter)
    {
        LOG_ERR("!!! missed packages: %i", (header[SERIAL_FRAME_COUNTER] - serial_interface->packet_down_counter));
        serial_interface->packet_down_counter = header[SERIAL_FRAME_COUNTER];
    }
    uint16_t calculated_crc = crc16_itu_t(0xffff, bytes, length);
    uint16_t received_crc = header[SERIAL_FRAME_CRC1] << 8 | header[SERIAL_FRAME_CRC2];
    if (calculated_crc != received_crc)
    {
        LOG_ERR("CRC not correct 0x%x != 0x%x", calculated_crc, received_crc);
        return false;
    }
    return true;
}

static void process_rx_buffer(struct k_work *work)
{
    serial_interface_t *serial_interface = CONTAINER_OF(work, serial_interface_t, process_rx_work);
    uint8_t *data = serial_interface->rx_buffer;
    uint8_t size = serial_interface->bytes_received;
    uint8_t index = 0;
    while (index < serial_interface->bytes_received)
    {
        if (size < SERIAL_FRAME_HEADER_SIZE)
        {
            LOG_ERR("Amount of received data is smaller than header size.");
            serial_interface->rx_buffer_state = BUFFER_STATE_IDLE;
            return;
        }
        if (data[index] != SERIAL_FRAME_SYNC_BYTE || data[index + 1] != SERIAL_FRAME_VERSION)
        {
            LOG_ERR("Received data is invalid 0x%x 0x%x.", data[index], data[index + 1]);
            index += 1;
            continue;
        }
        uint32_t payload_length = data[index + SERIAL_FRAME_SIZE];
        if (size < (SERIAL_FRAME_HEADER_SIZE + payload_length))
        {
            LOG_ERR("Amount of received data is smaller than packet size.");
            index += 1;
            continue;
        }
        if (!verify_payload(serial_interface, &data[index + SERIAL_FRAME_HEADER_SIZE], payload_length, data))
        {
            index += 1;
            continue;
            ;
        }
        LOG_DBG("Valid Data received!");
        if (data[index + SERIAL_FRAME_TYPE] == SERIAL_MESSAGE_TYPE_ALP_DATA && serial_interface->alp_handler)
        {
            serial_interface->alp_handler(serial_interface, &data[index + SERIAL_FRAME_HEADER_SIZE], payload_length);
        }
        else if (data[index + SERIAL_FRAME_TYPE] == SERIAL_MESSAGE_TYPE_PING_RESPONSE && serial_interface->ping_handler)
        {
            serial_interface->ping_handler(serial_interface, &data[index + SERIAL_FRAME_HEADER_SIZE], payload_length);
        }
        else if (data[index + SERIAL_FRAME_TYPE] == SERIAL_MESSAGE_TYPE_LOGGING && serial_interface->logging_handler)
        {
            serial_interface->logging_handler(serial_interface, &data[index + SERIAL_FRAME_HEADER_SIZE], payload_length);
        }
        else if (data[index + SERIAL_FRAME_TYPE] == SERIAL_MESSAGE_TYPE_PING_REQUEST)
        {
            serial_interface_transfer_bytes(serial_interface, &data[index + SERIAL_FRAME_HEADER_SIZE], payload_length,
                                            SERIAL_MESSAGE_TYPE_PING_RESPONSE);
        }
        else if (data[index + SERIAL_FRAME_TYPE] == SERIAL_MESSAGE_TYPE_REBOOTED)
        {
            LOG_INF("MODEM REBOOTED!");
        }
        index += SERIAL_FRAME_HEADER_SIZE + payload_length;
    }
    serial_interface->rx_buffer_state = BUFFER_STATE_IDLE;
}

static void flush_serial_interface_tx_buffer(struct k_work *work)
{
    serial_interface_t *serial_interface = CONTAINER_OF(work, serial_interface_t, tx_work);
    serial_interface->tx_active = false;
    int ret = uart_tx(serial_interface->uart, serial_interface->tx_buffer, serial_interface->bytes_to_transmit, SYS_FOREVER_US);
    LOG_INF("UART_TX_RET: %d", ret);
    // k_msleep(100);
    __ASSERT(ret == 0, "Error starting transmit %d", ret);
}

void uart_event_handler(const struct device *dev, struct uart_event *evt, void *user_data)
{
    serial_interface_t *serial_interface = (serial_interface_t *)user_data;
    int ret;
    switch (evt->type)
    {
    case UART_TX_DONE:
        LOG_DBG("TX_DONE");
        __ASSERT(serial_interface->bytes_to_transmit == evt->data.tx.len, "Not all bytes transmitted. %d bytes lost",
                 serial_interface->bytes_to_transmit - evt->data.tx.len);
        serial_interface->request_pending = false;
        serial_interface->tx_flush_busy = false;
        serial_interface->tx_active = false;
        serial_interface->bytes_to_transmit = 0;
        release_receiver(serial_interface);
        ret = k_work_submit(&serial_interface->state_machine_work);
        __ASSERT(ret >= 0, "Failed to submit state_machine_work %d", ret);
        break;
    case UART_TX_ABORTED:
        LOG_DBG("UART_TX_ABORTED");
        serial_interface->request_pending = false;
        serial_interface->tx_flush_busy = false;
        serial_interface->tx_active = false;
        serial_interface->bytes_to_transmit = 0;
        LOG_DBG("UART_TX_ABORTED");
        break;
    case UART_RX_RDY:
        LOG_DBG("RX_DONE");
        serial_interface->bytes_received = evt->data.rx.len;
        __ASSERT(evt->data.rx.offset == 0, "Expected an offset of 0, %d instead", evt->data.rx.offset);
        serial_interface->rx_active = false;
        if (serial_interface->bytes_received > 0)
        {
            serial_interface->rx_buffer_state = BUFFER_STATE_PROCESS;
            ret = k_work_submit(&serial_interface->process_rx_work);
            __ASSERT(ret >= 0, "Submitting process_rx_work failed with error %d", ret);
        }
        break;
    case UART_RX_BUF_REQUEST:
        /* We only work with one buffer */
        break;
    case UART_RX_BUF_RELEASED:
        LOG_DBG("UART_RX_BUF_RELEASED");
        break;
    case UART_RX_DISABLED:
        LOG_DBG("UART_RX_DISABLED");
        serial_interface->rx_active = false;
        if (serial_interface->rx_buffer_state == BUFFER_STATE_UART)
        {
            serial_interface->rx_buffer_state = BUFFER_STATE_IDLE;
        }
        release_receiver(serial_interface);
        break;
    case UART_RX_STOPPED:
        LOG_ERR("Rx stopped due to 0x%x. %d bytes were received => dropping", evt->data.rx_stop.reason, evt->data.rx_stop.data.len);
        serial_interface->rx_active = false;
        serial_interface->rx_buffer_state = BUFFER_STATE_IDLE;
        break;
    }
    if (serial_interface->shutdown && !serial_interface->rx_active && !serial_interface->tx_active)
    {
        serial_interface->shutdown = false;
        ret = pm_device_action_run(serial_interface->uart, PM_DEVICE_ACTION_SUSPEND);
        LOG_INF("PM_RET3: %d", ret);

        LOG_DBG("UART disabled (event)");
    }
}

static void execute_state_machine(struct k_work *work)
{
    serial_interface_t *serial_interface = CONTAINER_OF(work, serial_interface_t, state_machine_work);
    int ret;
    LOG_INF("STATE: %d\n", serial_interface->state);
    switch (serial_interface->state)
    {
    case STATE_REC:
        if (gpio_pin_get_dt(serial_interface->target_uart_state_pin))
            break;
        SWITCH_STATE(serial_interface->state, STATE_RESP);
        // Intentional fall through
    case STATE_RESP:
    {
        // response period completed, abort reception witch will trigger uart event handler
        uart_rx_disable(serial_interface->uart);
        if (serial_interface->request_pending)
        {
            SWITCH_STATE(serial_interface->state, STATE_RESP_PENDING_REQ);
            ret = k_work_submit(&serial_interface->state_machine_work);
            __ASSERT(ret >= 0, "Failed to submit state_machine_work %d", ret);
        }
        else
        {
            SWITCH_STATE(serial_interface->state, STATE_IDLE);
            gpio_pin_set_dt(serial_interface->uart_state_pin, 0);
            serial_interface_disable(serial_interface);
        }
        break;
    }
    case STATE_IDLE:
        if (gpio_pin_get_dt(serial_interface->target_uart_state_pin))
        {
            // Check if receive buffer is available otherwise just wait
            if (serial_interface->rx_buffer_state == BUFFER_STATE_IDLE)
            {
                // wake-up requested
                SWITCH_STATE(serial_interface->state, STATE_REC);
                listen(serial_interface);
            }
            else
            {
                ret = k_work_submit(&serial_interface->state_machine_work);
                __ASSERT(ret >= 0, "Failed to submit state_machine_work %d", ret);
            }
            break;
        }
        else if (serial_interface->request_pending)
        { // check if we are really requesting a start
            SWITCH_STATE(serial_interface->state, STATE_REQ_START);
            // fall-through to STATE_REQ_START!
        }
        else
        {
            break;
        }
    case STATE_REQ_START:
        // TODO timeout
        SWITCH_STATE(serial_interface->state, STATE_REQ_WAIT);
        gpio_pin_set_dt(serial_interface->uart_state_pin, 1); // wake-up receiver
        LOG_DBG("wake-up receiver");
        ret = k_work_submit(&serial_interface->state_machine_work);
        __ASSERT(ret >= 0, "Failed to submit state_machine_work %d", ret); // reschedule again to prevent sleeoping
        // in principle we could go to sleep but this will cause pin to float, this can be improved later
        break;
    case STATE_REQ_WAIT:
        if (gpio_pin_get_dt(serial_interface->target_uart_state_pin))
        {
            // receiver active
            SWITCH_STATE(serial_interface->state, STATE_REQ_BUSY);
            // fall-through to STATE_REQ_BUSY!
        }
        else
        {
            // TODO timeout
            ret = k_work_submit(&serial_interface->state_machine_work);
            __ASSERT(ret >= 0, "Failed to submit state_machine_work %d", ret); // reschedule again to prevent sleeoping
            // in principle we could go to sleep but this will cause pin to float, this can be improved later
            break;
        }
    case STATE_REQ_BUSY:
        if (serial_interface->request_pending)
        {
            // if receiver reacts fast this code is executed twice (once due to the fall through of the previous state and once scheduled by the
            // uart_int_cb) To be sure that serial_interface_enable is not called the second time when the transfer is already busy it is
            // guaranteed that the code is only executed once.
            if (!serial_interface->tx_flush_busy)
            {
                serial_interface->tx_flush_busy = true;
                serial_interface_enable(serial_interface);
                ret = k_work_submit(&serial_interface->tx_work);
                __ASSERT(ret >= 0, "Failed to queue Tx work %d", ret);
                LOG_INF("TX_WORK_RET: %d", ret);

                //
            }
        }
        else if (!gpio_pin_get_dt(serial_interface->target_uart_state_pin))
        {
            SWITCH_STATE(serial_interface->state, STATE_IDLE);
            ret = k_work_submit(&serial_interface->state_machine_work);
            __ASSERT(ret >= 0, "Failed to submit state_machine_work %d", ret);
        }
        else
        {
            ret = k_work_submit(&serial_interface->state_machine_work);
            __ASSERT(ret >= 0, "Failed to submit state_machine_work %d", ret);
            // keep active until target reacts
        }
        break;
    case STATE_RESP_PENDING_REQ:
        __ASSERT_NO_MSG(serial_interface->request_pending);
        // response period completed, initiate pending request by switching to REQ_START
        __ASSERT_NO_MSG(!gpio_pin_get_dt(serial_interface->target_uart_state_pin));
        gpio_pin_set_dt(serial_interface->uart_state_pin, 0);
        SWITCH_STATE(serial_interface->state, STATE_REQ_START);
        ret = k_work_submit(&serial_interface->state_machine_work);
        __ASSERT(ret >= 0, "Failed to submit state_machine_work %d", ret);
        break;
    default:
        LOG_ERR("unexpected state %i\n", serial_interface->state);
        __ASSERT_NO_MSG(false);
    }
}

static void target_state_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    // do not read GPIO level here in interrupt context (GPIO clock might not be enabled yet), execute state machine instead
    LOG_DBG("Modem pin interrupt callback");
    int ret;
    // k_msleep(100);
    serial_interface_t *serial_interface = CONTAINER_OF(cb, serial_interface_t, target_state_callback);
    LOG_DBG("INT %d", gpio_pin_get_dt(serial_interface->target_uart_state_pin));
    ret = k_work_submit(&serial_interface->state_machine_work);
    __ASSERT(ret >= 0, "Failed to submit state_machine_work %d", ret);
}

serial_interface_t *serial_interface_init(const struct device *uart, const struct gpio_dt_spec *uart_state_pin,
                                          const struct gpio_dt_spec *target_uart_state_pin)
{
    int ret;
    __ASSERT_NO_MSG(serial_interface_count < SERIAL_INTERFACE_COUNT);
    serial_interface_t *serial_interface = &serial_interfaces[serial_interface_count];
    serial_interface_count++;
    uart_callback_set(uart, uart_event_handler, serial_interface);
    serial_interface->uart = uart;
    // Uart by default enabled by OS => disable
    ret = pm_device_action_run(serial_interface->uart, PM_DEVICE_ACTION_SUSPEND);
    LOG_INF("PM_RET4: %d", ret);

    serial_interface->uart_state_pin = uart_state_pin;
    serial_interface->target_uart_state_pin = target_uart_state_pin;
    serial_interface->rx_buffer_state = BUFFER_STATE_IDLE;

    k_work_init(&serial_interface->process_rx_work, process_rx_buffer);
    k_work_init(&serial_interface->tx_work, flush_serial_interface_tx_buffer);
    k_work_init(&serial_interface->state_machine_work, execute_state_machine);

    gpio_pin_configure_dt(uart_state_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(target_uart_state_pin, GPIO_INPUT);
    gpio_init_callback(&serial_interface->target_state_callback, target_state_cb, BIT(target_uart_state_pin->pin));
    gpio_add_callback(target_uart_state_pin->port, &serial_interface->target_state_callback);
    gpio_pin_interrupt_configure_dt(target_uart_state_pin, GPIO_INT_EDGE_BOTH);
    if (gpio_pin_get_dt(target_uart_state_pin))
    {
        ret = k_work_submit(&serial_interface->state_machine_work);
        __ASSERT(ret >= 0, "Failed to submit state_machine_work %d", ret);
    }
    return serial_interface;
}

void serial_interface_transfer_bytes(serial_interface_t *serial_interface, const uint8_t *bytes, uint8_t length, serial_message_type_t type)
{
    int ret;
    LOG_DBG("serial_interface_transfer_bytes");
    if (serial_interface->tx_flush_busy)
    {
        __ASSERT(0, "Transmit in progress, data will be dropped.");
        return;
    }
    if ((SERIAL_FRAME_HEADER_SIZE + length) > (UART_TX_BUF_SIZE - serial_interface->bytes_to_transmit))
    {
        __ASSERT(0, "Data will not fit in buffer, data will be dropped %d > %d.", (SERIAL_FRAME_HEADER_SIZE + length),
                 (UART_TX_BUF_SIZE - serial_interface->bytes_to_transmit));
        return;
    }
    uint8_t header[SERIAL_FRAME_HEADER_SIZE];
    uint16_t crc = crc16_itu_t(0xffff, bytes, length);

    serial_interface->packet_up_counter++;
    header[0] = SERIAL_FRAME_SYNC_BYTE;
    header[1] = SERIAL_FRAME_VERSION;

    header[SERIAL_FRAME_COUNTER] = serial_interface->packet_up_counter;
    header[SERIAL_FRAME_TYPE] = type;
    header[SERIAL_FRAME_SIZE] = length;
    header[SERIAL_FRAME_CRC1] = (crc >> 8) & 0x00FF;
    header[SERIAL_FRAME_CRC2] = crc & 0x00FF;

    unsigned int lock_key = irq_lock();
    memcpy(&serial_interface->tx_buffer[serial_interface->bytes_to_transmit], header, SERIAL_FRAME_HEADER_SIZE);
    serial_interface->bytes_to_transmit += SERIAL_FRAME_HEADER_SIZE;
    memcpy(&serial_interface->tx_buffer[serial_interface->bytes_to_transmit], bytes, length);
    serial_interface->bytes_to_transmit += length;
    serial_interface->request_pending = true;
    irq_unlock(lock_key);
    ret = k_work_submit(&serial_interface->state_machine_work);
    __ASSERT(ret >= 0, "Failed to submit state_machine_work %d", ret);
}

void serial_interface_register_handler(serial_interface_t *serial_interface, cmd_handler_t cmd_handler, serial_message_type_t type)
{
    switch (type)
    {
    case SERIAL_MESSAGE_TYPE_ALP_DATA:
        serial_interface->alp_handler = cmd_handler;
        break;
    case SERIAL_MESSAGE_TYPE_PING_RESPONSE:
        serial_interface->ping_handler = cmd_handler;
        break;
    case SERIAL_MESSAGE_TYPE_LOGGING:
        serial_interface->logging_handler = cmd_handler;
        break;
    default:
        LOG_ERR("Message handler not supported for given type.");
        break;
    }
}

void serial_interface_unregister_handler(serial_interface_t *serial_interface, serial_message_type_t type)
{
    serial_interface_register_handler(serial_interface, NULL, type);
}