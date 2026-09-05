#pragma once

enum device_state_t {
	DEVICE_STATE_GLOWSTICK_GREEN,
	DEVICE_STATE_GLOWSTICK_PURPLE,
	DEVICE_STATE_GLOWSTICK_LIME,
	DEVICE_STATE_GLOWSTICK_BLUE,
	DEVICE_STATE_GLOWSTICK_RED,
	DEVICE_STATE_GLOWSTICK_WHITE,
	DEVICE_STATE_GLOWSTICK_BLAST,
	DEVICE_STATE_SWING_MODE,
	DEVICE_STATE_OSC_SWING_MODE
};

extern volatile enum device_state_t device_state;

void next_device_state(void);

/* Jumps directly to a given state (clamped to the valid range) instead of
 * cycling one step at a time - used by the serial console so a specific
 * mode (e.g. DEVICE_STATE_OSC_SWING_MODE) can be selected in one command
 * without needing physical access to the button. */
void state_machine_force(enum device_state_t state);