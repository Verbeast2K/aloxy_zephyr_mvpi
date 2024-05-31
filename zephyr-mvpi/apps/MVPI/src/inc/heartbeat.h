#ifndef HEARTBEAT_FILE_H
#define HEARTBEAT_FILE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t version;
    uint8_t counter;
    uint8_t uplinks;
    uint8_t ACK;
} __attribute__((__packed__)) heartbeat_file_t;

typedef struct
{
    uint8_t request_tag[2];
    uint8_t indirect_forward[2];
    uint8_t header[4];
    heartbeat_file_t heartbeat_file;
} heartbeat_payload_t;

void heartbeat_counter_increment();

void heartbeat_uplinks_increment();

void heartbeat_uplinks_clear();

void heartbeat_ACK_received(bool received);

heartbeat_file_t *heartbeat_initialize();

#endif // HEARTBEAT_FILE_H