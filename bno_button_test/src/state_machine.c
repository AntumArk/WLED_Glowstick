#include "state_machine.h"

volatile enum device_state_t device_state = DEVICE_STATE_GLOWSTICK_GREEN;

void next_device_state(void)
{
	device_state = (enum device_state_t)((device_state + 1) % (DEVICE_STATE_OSC_SWING_MODE + 1));
}

void state_machine_force(enum device_state_t state)
{
	if (state < DEVICE_STATE_GLOWSTICK_GREEN) state = DEVICE_STATE_GLOWSTICK_GREEN;
	if (state > DEVICE_STATE_OSC_SWING_MODE) state = DEVICE_STATE_OSC_SWING_MODE;
	device_state = state;
}