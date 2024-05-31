/*
 * ALOXY CONFIDENTIAL
 * __________________
 *  [2017] - [2022] Aloxy nv
 * All Rights Reserved.
 *  NOTICE: All information contained herein is, and remains
 * the property of aloxy nv and its suppliers,
 * if any. The intellectual and technical concepts contained
 * herein are proprietary to Aloxy nv
 * and its suppliers and may be covered by patents and
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Aloxy nv.
 */

#ifndef __HEARTBEAT_FILE_H
#define __HEARTBEAT_FILE_H

// application

// aloxy

// chips

// platform

// oss7

// other
// #include "errors.h"
// #include "stdint.h"

/**
 * @brief Initializes the heartbeat file on the filesystem
 *
 * @return error_t
 */
error_t heartbeat_file_initialize();

/**
 * @brief Sends the heartbeat to the modem. Heartbeat will be rescheduled every {heartbeat_interval} seconds until heartbeat_file_stop is called.
 * The heartbeat counter starts at 0 and is increased by one each time it is sent. Upon turnaround (> 255), the heartbeat counter is reset to 1.
 */
void heartbeat_file_send();

/**
 * @brief Causes triggered_manually flag of the next heartbeat to be set.
 *
 */
void heartbeat_file_set_manually_triggerd();

/**
 * @brief Stops the automatic rescheduling of the heartbeat
 *
 */
void heartbeat_file_stop();

/**
 * @brief Resets the heartbeat file. This causes the counter to be reset to 0.
 *
 */
void heartbeat_file_reset();

/**
 * @brief Print the contents of the heartbeat file
 *
 */
void heartbeat_file_print();

/**
 * @brief Set the heartbeat interval object
 *
 * @param interval
 */
void set_heartbeat_interval(uint32_t interval);

#endif