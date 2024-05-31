#ifndef ZEPHYR_INCLUDE_STATE_MACHINE_H
#define ZEPHYR_INCLUDE_STATE_MACHINE_H

/**
 *  @def state_machine_init()
 * @brief Function to populate the state table
 */
void state_machine_init();
// const struct smf_state state_machine_init();

/**
 *  @def state_machine_run()
 * @brief Function to run the state machine
 */
void state_machine_run();

#endif // ZEPHYR_INCLUDE_STATE_MACHINE_H