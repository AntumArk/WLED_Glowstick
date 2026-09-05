#pragma once

/* Serves a tiny HTML form at http://<ap-ip>/ (default http://192.168.4.1/)
 * so a phone/laptop connected to the "Glowstick OSC" hotspot can enter real
 * Wi-Fi credentials without any app beyond a browser. Submitting the form
 * saves the SSID/password to NVS and reboots the device, which will then
 * try to join that network on next boot (see wifi_manager.c). Only useful
 * (and only started) while the device is running as an access point. */
void wifi_web_config_start(void);
void wifi_web_config_stop(void);
