# BNO055 + Button Test

Tiny standalone ESP-IDF test app for the XIAO ESP32-C6 wiring.

## Wiring

- BNO055 SDA -> D4
- BNO055 SCL -> D5
- BNO055 VCC -> 3V3
- BNO055 GND -> GND
- Button -> D0 with pull-down

## What it does

- Prints BNO055 calibration state, linear acceleration, gyro, and quaternion over serial.
- Runs the "glowstick" LED mode (bouncing ball / color cycle / swing peak flash),
  cycled with a short button press.
- Sends swing/hit events and (optionally) live accelerometer data as
  **Open Sound Control (OSC)** messages over Wi-Fi/UDP - see below.
- Uses no WLED code.

## Build / flash

```bash
cd bno_button_test
/home/zinc/.platformio/penv/bin/platformio run
/home/zinc/.platformio/penv/bin/platformio run -t upload
/home/zinc/.platformio/penv/bin/platformio device monitor
```

## Wi-Fi bring-up

The device tries to join a configured Wi-Fi network on boot. If none is
configured yet, or the join fails, it falls back to running as its own
open access point named **`Glowstick OSC`** at `192.168.4.1`.

### Configuring your own network

While the device is running as the `Glowstick OSC` access point:

1. Join that Wi-Fi network from a phone or laptop.
2. Open `http://192.168.4.1/` in a browser.
3. Enter your Wi-Fi SSID/password and submit.

The device saves the credentials to NVS and reboots, then tries to join
that network. On success it will no longer advertise the `Glowstick OSC`
hotspot - check your router's client list (or the serial log) for its new
IP address. See [`src/wifi_manager.c`](src/wifi_manager.c) /
[`src/wifi_web_config.c`](src/wifi_web_config.c).

> **Hardware note:** the XIAO ESP32-C6 has an RF antenna switch (GPIO3 =
> switch enable, GPIO14 = antenna select) that isn't mentioned anywhere in
> ESP-IDF's own docs - only in Seeed's Arduino-framework example. Without
> explicitly driving both pins low at boot, the radio can still weakly
> transmit (so AP mode "works" at very close range) but station-mode
> scanning finds **zero** networks. `wifi_manager_init()` configures this
> before touching Wi-Fi at all; see `configure_onboard_antenna()` in
> [`src/wifi_manager.c`](src/wifi_manager.c).

## OSC (Open Sound Control)

The device implements the wire format from
[the OSC spec](https://en.wikipedia.org/wiki/Open_Sound_Control) directly
over UDP - it is a plain OSC peer, not tied to any particular app. It
listens on **UDP port 9000** for any inbound OSC packet and "registers"
the sender as the destination for its own outgoing messages (sent to UDP
port 9001 by default). This means any OSC-capable tool (TouchOSC, Pure
Data, Max/MSP, `python-osc`, `oscsend` from liblo, ...) can start
receiving simply by sending the device one OSC packet:

```bash
oscsend <device-ip> 9000 /glowstick/register
```

See [`src/osc.h`](src/osc.h) / [`src/osc.c`](src/osc.c) for the transport
and [`src/osc_config.h`](src/osc_config.h) / [`src/osc_config.c`](src/osc_config.c)
for the registration/config layer.

### Output (device -> host)

| Address              | Args        | Meaning                                                        |
|----------------------|-------------|-----------------------------------------------------------------|
| `/glowstick/accel`    | `,fff` (m/s²)  | Live linear acceleration X/Y/Z, sent periodically when streaming is enabled |
| `/glowstick/orientation` | `,ffff` | Absolute orientation quaternion X/Y/Z/W |
| `/glowstick/gyro`     | `,fff` (deg/s) | Angular velocity X/Y/Z |
| `/glowstick/mag`      | `,fff` (uT)    | Magnetometer X/Y/Z |
| `/glowstick/gravity`  | `,fff` (m/s²)  | Gravity vector X/Y/Z |
| `/glowstick/calib`    | `,iiii`        | System/gyro/accelerometer/magnetometer calibration, each 0-3 |
| `/glowstick/ndof`     | `,iii`         | Operation mode/system status/system error; healthy NDOF_FMC_OFF is 11/5/0 |
| `/glowstick/selftest` | `,iiii`        | MCU/gyro/accelerometer/magnetometer power-on self-test, each 0/1 |

The seven high-rate IMU messages (`accel` through `ndof`) are sent together
in one immediate OSC bundle per sample. This preserves the individual OSC
addresses while reducing UDP packet and socket-call overhead. Status messages
remain ordinary standalone OSC messages.

### Configuration (host -> device)

A few addresses are recognized as configuration commands, applied
immediately and persisted to NVS (survives power cycles):

| Address                          | Args | Meaning                                                    |
|-----------------------------------|------|--------------------------------------------------------------|
| `/glowstick/register`             | none | Registers the sender's IP as the streaming target            |
| `/glowstick/config/port`          | `,i` | Sets the UDP port used for outgoing messages                 |
| `/glowstick/config/stream`        | `,i` | 0/1: enable or disable the periodic `/glowstick/accel` stream |
| `/glowstick/config/streamperiod`  | `,i` | Stream period in milliseconds (10-10000ms; 10ms = 100Hz)     |

Any other address is ignored by this layer.

### Manual testing without an OSC app

A dependency-free Python sender/tester (`register`/`stream on`/`stream off`/`port`
commands) is useful for testing without installing a full OSC library - ask
for `osc_test.py` if you don't already have a copy handy.
