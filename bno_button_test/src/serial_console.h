#pragma once

/* Lightweight line-based command console over the USB-Serial/JTAG stdio
 * link, so the device's mode (and a few OSC config knobs) can be driven
 * from a serial terminal when the physical button isn't reachable (e.g.
 * the board is on a bench/desk connected only via USB). See
 * serial_console.c for the list of supported commands ("help" prints
 * them from the running device too). */
void start_serial_console_task(void);
