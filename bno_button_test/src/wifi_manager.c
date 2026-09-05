#include "wifi_manager.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "lwip/inet.h"
#include "mdns.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_manager";
static const char *NVS_NAMESPACE = "wificfg";
static const char *WIFI_HOSTNAME = "glowstick-osc";

/* XIAO ESP32C6 hardware quirk (undocumented in ESP-IDF, only in Seeed's
 * Arduino examples): the board has an RF switch between the onboard
 * ceramic antenna and an external u.FL connector. GPIO3 must be driven low
 * to enable the switch's control input, and GPIO14 then selects the
 * antenna (low = onboard ceramic, high = external). Without this, RX
 * sensitivity is effectively crippled -- AP-mode beacons can still leak
 * out at very close range, but station-mode scanning finds nothing.
 * See: https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/#rf-switch
 */
#define WIFI_RF_SWITCH_ENABLE_GPIO GPIO_NUM_3
#define WIFI_RF_ANTENNA_SELECT_GPIO GPIO_NUM_14

static void configure_onboard_antenna(void) {
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << WIFI_RF_SWITCH_ENABLE_GPIO) | (1ULL << WIFI_RF_ANTENNA_SELECT_GPIO),
      .mode = GPIO_MODE_OUTPUT,
  };
  gpio_config(&io_conf);
  gpio_set_level(WIFI_RF_SWITCH_ENABLE_GPIO, 0); /* enable RF switch control */
  gpio_set_level(WIFI_RF_ANTENNA_SELECT_GPIO, 0); /* select onboard ceramic antenna */
}

static void configure_hostname(esp_netif_t *netif) {
  if (netif == NULL) return;

  const esp_err_t err = esp_netif_set_hostname(netif, WIFI_HOSTNAME);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "failed to set hostname on netif: %s", esp_err_to_name(err));
  }
}

/* Open SoftAP so any phone/laptop can join without needing a shared secret
 * for a throwaway test rig; document this clearly if this ever leaves the
 * test-bench context. */
#define WIFI_AP_SSID "Glowstick OSC"
#define WIFI_AP_CHANNEL 6
#define WIFI_AP_MAX_CONN 4

#define WIFI_STA_CONNECT_TIMEOUT_MS 20000U
#define WIFI_STA_MAX_RETRIES 5U

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static EventGroupHandle_t wifi_event_group;
static esp_netif_t *sta_netif;
static esp_netif_t *ap_netif;
static bool wifi_initialized = false;
static bool mdns_started = false;
static bool ap_mode = false;
static char ip_str[16] = "0.0.0.0";
static uint32_t ip_addr_nbo = 0;
static uint32_t netmask_nbo = 0;
static uint32_t sta_retry_count = 0;

static void start_mdns(void) {
  if (mdns_started) return;

  esp_err_t err = mdns_init();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "failed to init mDNS: %s", esp_err_to_name(err));
    return;
  }

  err = mdns_hostname_set(WIFI_HOSTNAME);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "failed to set mDNS hostname: %s", esp_err_to_name(err));
    mdns_free();
    return;
  }

  err = mdns_instance_name_set("Glowstick OSC");
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "failed to set mDNS instance name: %s", esp_err_to_name(err));
    mdns_free();
    return;
  }

  mdns_started = true;
  ESP_LOGI(TAG, "mDNS started: %s.local", WIFI_HOSTNAME);
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  (void)arg;
  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
    ip_addr_nbo = event->ip_info.ip.addr;
    netmask_nbo = event->ip_info.netmask.addr;
    ESP_LOGI(TAG, "station got ip: %s", ip_str);
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  (void)arg;
  if (event_base != WIFI_EVENT) return;

  if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
    if (sta_retry_count < WIFI_STA_MAX_RETRIES) {
      sta_retry_count++;
      ESP_LOGW(TAG, "station disconnected (reason=%d), retry %lu/%u", event->reason,
               (unsigned long)sta_retry_count, WIFI_STA_MAX_RETRIES);
      esp_wifi_connect();
    } else {
      ESP_LOGW(TAG, "station disconnected (reason=%d), giving up after %u retries", event->reason,
               WIFI_STA_MAX_RETRIES);
      xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
    }
  } else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
    ESP_LOGI(TAG, "station " MACSTR " joined our AP", MAC2STR(event->mac));
  } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
    ESP_LOGI(TAG, "station " MACSTR " left our AP", MAC2STR(event->mac));
  }
}

/* Reads a persisted station SSID/password (if any) from NVS. Nothing writes
 * these yet since this project's default flow is AP-only, but the storage
 * is here so a future OSC config message (or a one-off manual NVS write)
 * can enable STA mode without a firmware change. */
static bool load_sta_credentials(char *ssid, size_t ssid_len, char *pass, size_t pass_len) {
  nvs_handle_t handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return false;

  size_t required = ssid_len;
  esp_err_t err = nvs_get_str(handle, "ssid", ssid, &required);
  if (err != ESP_OK || ssid[0] == '\0') {
    nvs_close(handle);
    return false;
  }

  required = pass_len;
  err = nvs_get_str(handle, "pass", pass, &required);
  nvs_close(handle);
  if (err != ESP_OK) pass[0] = '\0';
  return true;
}

static bool try_connect_station(void) {
  char ssid[33] = {0};
  char pass[65] = {0};
  if (!load_sta_credentials(ssid, sizeof(ssid), pass, sizeof(pass))) return false;

  ESP_LOGI(TAG, "attempting to join configured network \"%s\"", ssid);
  sta_retry_count = 0;

  if (sta_netif == NULL) sta_netif = esp_netif_create_default_wifi_sta();
  configure_hostname(sta_netif);

  wifi_config_t wifi_config = {0};
  strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
  strlcpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
  /* WIFI_AUTH_OPEN as the *minimum* threshold means "accept whatever auth
   * mode the AP actually uses" (open, WPA, WPA2, WPA3, ...) instead of
   * rejecting the connection outright if it isn't at least WPA2-PSK. */
  wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  /* Diagnostic: scan first and log every AP we can actually see, so a
   * "no AP found" failure below is easy to tell apart from "found it but
   * auth/handshake failed" (bad password, incompatible security, etc). */
  esp_err_t scan_err = esp_wifi_scan_start(NULL, true);
  if (scan_err == ESP_OK) {
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    ESP_LOGI(TAG, "scan reports %u AP record(s) available", ap_count);
    if (ap_count > 16) ap_count = 16;
    wifi_ap_record_t ap_records[16];
    uint16_t ap_records_count = ap_count;
    esp_err_t get_err = esp_wifi_scan_get_ap_records(&ap_records_count, ap_records);
    if (get_err == ESP_OK) {
      ESP_LOGI(TAG, "scan found %u nearby network(s):", ap_records_count);
      for (uint16_t i = 0; i < ap_records_count; i++) {
        ESP_LOGI(TAG, "  \"%s\" rssi=%d authmode=%d", (const char *)ap_records[i].ssid, ap_records[i].rssi,
                 ap_records[i].authmode);
      }
    } else {
      ESP_LOGW(TAG, "esp_wifi_scan_get_ap_records failed: %d", get_err);
    }
  } else {
    ESP_LOGW(TAG, "esp_wifi_scan_start failed: %d", scan_err);
  }

  esp_wifi_connect();

  EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
                                          pdMS_TO_TICKS(WIFI_STA_CONNECT_TIMEOUT_MS));
  if (bits & WIFI_CONNECTED_BIT) return true;

  ESP_LOGW(TAG, "failed to join \"%s\", falling back to hotspot mode", ssid);
  ESP_ERROR_CHECK(esp_wifi_stop());
  return false;
}

static void start_access_point(void) {
  ap_mode = true;
  if (ap_netif == NULL) ap_netif = esp_netif_create_default_wifi_ap();
  configure_hostname(ap_netif);

  wifi_config_t wifi_config = {0};
  strlcpy((char *)wifi_config.ap.ssid, WIFI_AP_SSID, sizeof(wifi_config.ap.ssid));
  wifi_config.ap.ssid_len = strlen(WIFI_AP_SSID);
  wifi_config.ap.channel = WIFI_AP_CHANNEL;
  wifi_config.ap.max_connection = WIFI_AP_MAX_CONN;
  wifi_config.ap.authmode = WIFI_AUTH_OPEN;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  esp_netif_ip_info_t ip_info;
  esp_netif_get_ip_info(ap_netif, &ip_info);
  esp_ip4addr_ntoa(&ip_info.ip, ip_str, sizeof(ip_str));
  ip_addr_nbo = ip_info.ip.addr;
  netmask_nbo = ip_info.netmask.addr;
  ESP_LOGI(TAG, "hotspot \"%s\" (open) started, ip=%s", WIFI_AP_SSID, ip_str);
}

void wifi_manager_init(void) {
  configure_onboard_antenna();

  if (!wifi_initialized) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    wifi_initialized = true;
  }

  ap_mode = false;
  strlcpy(ip_str, "0.0.0.0", sizeof(ip_str));
  ip_addr_nbo = 0;
  netmask_nbo = 0;
  xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

  if (!try_connect_station()) {
    start_access_point();
  }

  start_mdns();
}

void wifi_manager_stop(void) {
  if (!wifi_initialized) return;

  if (mdns_started) {
    mdns_free();
    mdns_started = false;
  }

  const esp_err_t err = esp_wifi_stop();
  if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED) {
    ESP_LOGW(TAG, "failed to stop Wi-Fi: %s", esp_err_to_name(err));
  }
  ap_mode = false;
  strlcpy(ip_str, "0.0.0.0", sizeof(ip_str));
  ip_addr_nbo = 0;
  netmask_nbo = 0;
}

bool wifi_manager_is_ap_mode(void) { return ap_mode; }

const char *wifi_manager_get_ip_str(void) { return ip_str; }

bool wifi_manager_is_same_subnet(uint32_t peer_ip) {
  if (peer_ip == 0 || ip_addr_nbo == 0 || netmask_nbo == 0) return false;

  const uint32_t local_ip = ntohl(ip_addr_nbo);
  const uint32_t peer = ntohl(peer_ip);
  const uint32_t mask = ntohl(netmask_nbo);
  return (local_ip & mask) == (peer & mask);
}

void wifi_manager_save_sta_credentials_and_reboot(const char *ssid, const char *pass) {
  nvs_handle_t handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
    ESP_LOGE(TAG, "failed to open NVS to save station credentials");
    return;
  }
  nvs_set_str(handle, "ssid", ssid);
  nvs_set_str(handle, "pass", pass ? pass : "");
  nvs_commit(handle);
  nvs_close(handle);

  ESP_LOGI(TAG, "saved station credentials for \"%s\", rebooting to join it", ssid);
  vTaskDelay(pdMS_TO_TICKS(500)); /* let the HTTP response flush before reset */
  esp_restart();
}

int8_t wifi_manager_get_rssi(void) {
  if (ap_mode) return 0;
  wifi_ap_record_t info;
  if (esp_wifi_sta_get_ap_info(&info) != ESP_OK) return 0;
  return info.rssi;
}
