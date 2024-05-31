#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem);

#define UART_NODE DT_NODELABEL(lpuart1)

#define BT1_NODE DT_NODELABEL(button1)
#if !DT_NODE_HAS_STATUS(BT1_NODE, okay)
#error "Unsupported board: button1 devicetree node is not defined"
#endif

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BT1_NODE, gpios);
static struct gpio_callback button_cb_data;

uint8_t msg = 2;

const struct device *uart = DEVICE_DT_GET(UART_NODE);

void btn_work()
{
    LOG_INF("BTN_WORK\n");

    int ret;
    msg += 1;

    uart_poll_out(uart, msg);
    k_msleep(500);
    ret = uart_tx(uart, &msg, sizeof(msg), SYS_FOREVER_US);
    LOG_INF("%d\n", ret);
    LOG_INF("%d\n", msg);
}
K_WORK_DEFINE(ping, btn_work);

void btn_pressed(const struct device *dev, struct gpio_callback *cb,
                 uint32_t pins)
{
    k_work_submit(&ping);
}

int main()
{
    int ret;

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
}