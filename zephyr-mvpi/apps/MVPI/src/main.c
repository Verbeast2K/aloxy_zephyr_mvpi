/*
	Combination of an input handler function and the Zephyr SMF
	to make the MVPI state machine that switches states depending
	on the current state and the type of input
*/

#include "inc/state_machine.h"
#include "inc/modem.h"

int main(void)
{
	/* Initialize the state machine */
	state_machine_init();

	/* Initialize serial interface */
	serial_interface_initialize();

	/* Run the state machine */
	state_machine_run();
}