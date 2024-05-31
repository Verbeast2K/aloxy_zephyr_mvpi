// #include <zephyr/kernel.h>

#include "inc/heartbeat.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(heartbeat, CONFIG_HEARTBEAT_LOG_LEVEL);

#define HEARTBEAT_VERSION 10

heartbeat_file_t heartbeat_file;

void heartbeat_counter_increment()
{
    heartbeat_file.counter += 1;
}

void heartbeat_uplinks_increment()
{
    heartbeat_file.uplinks += 1;
}

void heartbeat_uplinks_clear()
{
    heartbeat_file.uplinks = 0;
}

void heartbeat_ACK_received(bool received)
{
    if (received)
    {
        heartbeat_uplinks_clear();
        heartbeat_file.ACK = 1;
    }
    else if (!received)
    {
        heartbeat_file.ACK = 0;
    }
}

heartbeat_file_t *heartbeat_initialize()
{
    heartbeat_file.version = HEARTBEAT_VERSION;
    return &heartbeat_file;
}