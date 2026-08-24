#pragma once
/*
 * Persisted OSC configuration and "who to send to" registration.
 *
 * The device has no fixed destination compiled in: any OSC-capable tool
 * registers itself as the streaming target simply by sending the device a
 * single OSC message (any address), e.g.:
 *   oscsend <device-ip> 9000 /glowstick/register
 *
 * A few addresses are recognized as configuration commands (applied
 * immediately and persisted to NVS so they survive a power cycle):
 *
 *   Address                        Args   Meaning
 *   ------------------------------  -----  --------------------------------
 *   /glowstick/register              (none) Registers the sender as the
 *                                           streaming target, using the
 *                                           currently configured send port.
 *   /glowstick/config/port           ,i     Sets the UDP port used for
 *                                           outgoing messages (does not
 *                                           change the target host).
 *   /glowstick/config/stream         ,i     0/1: enable or disable the
 *                                           periodic /glowstick/accel
 *                                           stream.
 *   /glowstick/config/streamperiod   ,i     Stream period in milliseconds
 *                                           (10-10000ms).
 *
 * Any other address is ignored by this layer (but still reaches the
 * registered rx handler, if any other code wants to look at it).
 *
 * Outgoing (device -> host) addresses, sent to whoever last registered
 * (see glowstick_mode.c):
 *
 *   Address                  Args   Meaning
 *   ------------------------  -----  --------------------------------------
 *   /glowstick/accel           ,fff    Linear acceleration X/Y/Z (m/s^2)
 *   /glowstick/orientation     ,ffff   Absolute orientation quaternion X/Y/Z/W
 *   /glowstick/gyro            ,fff    Angular velocity X/Y/Z (deg/s)
 *   /glowstick/mag             ,fff    Magnetometer X/Y/Z (uT)
 *   /glowstick/gravity         ,fff    Gravity vector X/Y/Z (m/s^2)
 *   /glowstick/calib           ,iiii   BNO055 calibration status 0-3:
 *                                      system, gyro, accel, mag
 *   /glowstick/ndof            ,iii    BNO055 operation mode, fusion system
 *                                      status, and system error. Healthy
 *                                      NDOF_FMC_OFF values are 11, 5, 0.
 *   /glowstick/battery         ,ff     Pack voltage (V), charge percent (0-100)
 *   /glowstick/temp            ,f      BNO055 die temperature (degrees C)
 *   /glowstick/rssi            ,i      Wi-Fi signal strength (dBm), 0 if AP mode
 *   /glowstick/selftest        ,iiii   BNO055 power-on self-test pass/fail (1/0):
 *                                      MCU, gyro, accel, mag. If mag=0, the
 *                                      magnetometer failed self-test and
 *                                      /glowstick/mag will always read zero
 *                                      regardless of firmware.
 *   /glowstick/magprobe        ,fff    One-time raw magnetometer sample (uT)
 *                                      captured in standalone MAGONLY mode.
 *
 * /glowstick/accel, /glowstick/orientation, /glowstick/gyro,
 * /glowstick/mag, /glowstick/gravity, /glowstick/calib and
 * /glowstick/ndof are all sent
 * together at the configured stream period, gated by
 * /glowstick/config/stream. /glowstick/battery,
 * /glowstick/temp, /glowstick/rssi and /glowstick/selftest are sent every
 * 2 seconds unconditionally (as long as a target is registered).
 */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t target_ip;   /* network byte order; 0 = none registered yet */
  uint16_t target_port; /* UDP port used for outgoing messages */
  bool stream_enabled;
  uint16_t stream_period_ms;
} osc_config_t;

/* Loads persisted config from NVS (or compiled-in defaults on first boot),
 * re-registers any previously-known target with the OSC transport, and
 * registers this module as the OSC rx handler. */
void osc_config_init(void);

const osc_config_t *osc_config_get(void);
