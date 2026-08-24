#pragma once
/*
 * Wi-Fi bring-up for the OSC transport.
 *
 * If a station SSID is configured (see wifi_manager.c), the device tries to
 * join that network for WIFI_STA_CONNECT_TIMEOUT_MS. Since this is a
 * standalone test rig with no display/keyboard to enter credentials, the
 * simplest and most reliable path - and what's used by default here - is to
 * skip straight to acting as its own Wi-Fi access point ("hotspot") so any
 * phone/laptop can join it directly and send/receive OSC without needing to
 * share home network credentials with the device at all.
 */

#include <stdbool.h>
#include <stdint.h>

/* Brings up Wi-Fi: attempts station mode if an SSID is configured, falling
 * back to (or starting directly in, if no SSID is configured) SoftAP mode
 * named WIFI_AP_SSID. Blocks until an IP is available (own IP in AP mode,
 * or DHCP-assigned IP in STA mode). */
void wifi_manager_init(void);

/* True if the device ended up running as its own access point rather than
 * joining an existing network. */
bool wifi_manager_is_ap_mode(void);

/* Device's own IP address as a dotted-quad string ("192.168.4.1" in AP
 * mode, or the DHCP-assigned address in STA mode). */
const char *wifi_manager_get_ip_str(void);

/* Persists a station SSID/password to NVS (namespace "wificfg", keys
 * "ssid"/"pass") and reboots so wifi_manager_init() re-attempts a station
 * join on next boot. Used by the AP-mode config web page. */
void wifi_manager_save_sta_credentials_and_reboot(const char *ssid, const char *pass);

/* Current Wi-Fi signal strength in dBm (typically -30 to -90), read from
 * the AP this device is associated with in station mode. Returns 0 if not
 * connected in station mode (e.g. running as its own access point). */
int8_t wifi_manager_get_rssi(void);
