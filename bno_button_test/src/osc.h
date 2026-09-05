#pragma once
/*
 * Open Sound Control (OSC) transport over UDP.
 *
 * Implements the wire format described at
 * https://en.wikipedia.org/wiki/Open_Sound_Control : each message is an
 * OSC Address Pattern string (e.g. "/glowstick/hit"), followed by an OSC
 * Type Tag string ("," plus one letter per argument), followed by the
 * arguments themselves in big-endian binary form. Both strings are
 * null-terminated and zero-padded to a 4-byte boundary, and each argument
 * is padded/aligned to 4 bytes, per spec. This implementation supports the
 * two most commonly used argument types: 'i' (int32) and 'f' (float32).
 *
 * The device is a plain OSC peer over UDP, not a client of any particular
 * app: it listens on OSC_LISTEN_PORT for any inbound OSC packet, and
 * "registers" the sender's IP address as the destination for its own
 * outgoing hit/motion messages (sent to OSC_DEFAULT_SEND_PORT, or whatever
 * port was last set via /glowstick/config/port - see osc_config.h). This
 * means any OSC-capable tool (TouchOSC, Pure Data, Max/MSP, python-osc,
 * `oscsend` from liblo, ...) can start receiving simply by sending the
 * device a single OSC packet, e.g.:
 *   oscsend <device-ip> 9000 /glowstick/register
 */

#include <stdbool.h>
#include <stdint.h>

#define OSC_LISTEN_PORT 9000
#define OSC_DEFAULT_SEND_PORT 9001
#define OSC_BUNDLE_CAPACITY 384
#define STATUS_STREAM_PERIOD_MS 2000U

typedef struct {
    uint8_t data[OSC_BUNDLE_CAPACITY];
    uint16_t length;
} osc_bundle_t;

/* Called for every OSC message received on OSC_LISTEN_PORT, with the
 * address pattern, the raw argument bytes (still in big-endian wire
 * format, arg_count 'i'/'f' values only) and the sender's address (so the
 * config layer can "register" it as the streaming target). */
typedef void (*osc_rx_cb_t)(const char *address, const int32_t *int_args, uint8_t int_arg_count,
                             const float *float_args, uint8_t float_arg_count, uint32_t sender_ip,
                             uint16_t sender_port);

/* Starts the UDP socket, the receive task, and registers a low-level
 * dispatcher; osc_set_rx_handler() below is what higher layers use. */
void osc_task_init(void);

void osc_set_rx_handler(osc_rx_cb_t cb);

/* Sends `/<address>` followed by `count` 'f' (float32) arguments, in
 * order, to the currently registered target (a no-op, silently, if no
 * target has registered yet). `count` is capped at 8. This is the
 * generic building block osc_send_float1()/osc_send_float3() below wrap;
 * used directly for anything with a different argument count (e.g. a
 * 4-float quaternion). */
void osc_send_floats(const char *address, const float *values, uint8_t count);

/* Same as osc_send_floats() but for 'i' (int32) arguments, e.g. sensor
 * calibration status nibbles. */
void osc_send_ints(const char *address, const int32_t *values, uint8_t count);

/* Sends `/<address> ,f <value>` to the currently registered target. */
void osc_send_float1(const char *address, float value);

/* Sends `/<address> ,fff <x> <y> <z>` to the currently registered target. */
void osc_send_float3(const char *address, float x, float y, float z);

/* Builds and sends a bounded OSC bundle with the immediate-execution timetag. */
void osc_bundle_init(osc_bundle_t *bundle);
bool osc_bundle_add_floats(osc_bundle_t *bundle, const char *address, const float *values, uint8_t count);
bool osc_bundle_add_ints(osc_bundle_t *bundle, const char *address, const int32_t *values, uint8_t count);
void osc_bundle_send(const osc_bundle_t *bundle);

/* Updates the registered target (host, port) that outgoing messages are
 * sent to; used by osc_config.c when a /glowstick/register or
 * /glowstick/config/port message arrives. */
void osc_set_target(uint32_t ip, uint16_t port);
bool osc_has_target(void);
void stream_status_over_osc(uint32_t now);
void osc_tx_task(void *arg);
void osc_parse_and_dispatch(const uint8_t *buf, uint16_t len, uint32_t sender_ip, uint16_t sender_port);
void stream_imu_over_osc(uint32_t now);
// Initiates wifi connections
void osc_init(void);
void osc_off(void);