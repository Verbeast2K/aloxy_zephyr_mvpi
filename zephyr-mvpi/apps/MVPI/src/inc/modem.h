#ifndef MODEM_H
#define MODEM_H

// #include <stdint.h>

/**
 *  @def serial_interface_init()
 * @brief Function to initialise serial interface and register alp_handler
 */
void serial_interface_initialize();

/**
 *  @def serial_interface_start()
 * @brief Function to start serial interface communication by sending the start command
 */
void serial_interface_start();

/**
 *  @def serial_interface_stop()
 * @brief Function to stop serial interface communication by sending the stop command
 */
void serial_interface_stop();

// /**
//  *  @def serial_interface_tx()
//  * @brief Function to transmit data over serial interface
//  */
// void serial_interface_tx(uint8_t buf);

#endif // MODEM_H