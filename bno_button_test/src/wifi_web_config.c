#include "wifi_web_config.h"

#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"

#include "wifi_manager.h"

static const char *TAG = "wifi_web_config";
static httpd_handle_t server = NULL;

static const char INDEX_HTML[] =
    "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<title>Glowstick Wi-Fi setup</title></head><body style=\"font-family:sans-serif;max-width:360px;margin:2em "
    "auto\">"
    "<h2>Glowstick Wi-Fi setup</h2>"
    "<p>Enter the Wi-Fi network you want the Glowstick to join. It will reboot and, on success, leave this "
    "hotspot.</p>"
    "<form method=\"POST\" action=\"/save\">"
    "<label>SSID<br><input name=\"ssid\" maxlength=\"32\" required style=\"width:100%\"></label><br><br>"
    "<label>Password<br><input name=\"pass\" type=\"password\" maxlength=\"64\" style=\"width:100%\"></label><br><br>"
    "<button type=\"submit\">Save &amp; reboot</button>"
    "</form>"
    "<p>If the network isn't reachable, Glowstick falls back to its own \"Glowstick OSC\" hotspot again after a few "
    "seconds.</p>"
    "</body></html>";

static esp_err_t index_get_handler(httpd_req_t *req) {
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

/* Minimal application/x-www-form-urlencoded decoder: '+' -> space, '%XX' ->
 * byte. Decodes in place; out must be at least as large as the encoded
 * input. */
static void url_decode(char *s) {
  char *out = s;
  while (*s) {
    if (*s == '+') {
      *out++ = ' ';
      s++;
    } else if (*s == '%' && s[1] && s[2]) {
      char hex[3] = {s[1], s[2], '\0'};
      *out++ = (char)strtol(hex, NULL, 16);
      s += 3;
    } else {
      *out++ = *s++;
    }
  }
  *out = '\0';
}

/* Finds "key=value" within an x-www-form-urlencoded body (fields separated
 * by '&'), URL-decodes the value into out, and returns true if found. */
static bool extract_field(const char *body, const char *key, char *out, size_t out_len) {
  size_t key_len = strlen(key);
  const char *p = body;
  while (p && *p) {
    if (strncmp(p, key, key_len) == 0 && p[key_len] == '=') {
      const char *value_start = p + key_len + 1;
      const char *amp = strchr(value_start, '&');
      size_t value_len = amp ? (size_t)(amp - value_start) : strlen(value_start);
      if (value_len >= out_len) value_len = out_len - 1;
      memcpy(out, value_start, value_len);
      out[value_len] = '\0';
      url_decode(out);
      return true;
    }
    p = strchr(p, '&');
    if (p) p++;
  }
  return false;
}

static esp_err_t save_post_handler(httpd_req_t *req) {
  char body[256] = {0};
  size_t total = req->content_len < sizeof(body) - 1 ? req->content_len : sizeof(body) - 1;
  int received = httpd_req_recv(req, body, total);
  if (received <= 0) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  body[received] = '\0';

  char ssid[64] = {0};
  char pass[128] = {0};
  if (!extract_field(body, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_send(req, "SSID is required", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
  extract_field(body, "pass", pass, sizeof(pass));

  ESP_LOGI(TAG, "received new station credentials for \"%s\" via web config", ssid);

  static const char *saved_html =
      "<!DOCTYPE html><html><body style=\"font-family:sans-serif;max-width:360px;margin:2em auto\">"
      "<h2>Saved</h2><p>Glowstick is rebooting and will try to join that network.</p></body></html>";
  httpd_resp_send(req, saved_html, HTTPD_RESP_USE_STRLEN);

  /* Credentials are saved and the device rebooted from here; the response
   * above is queued for the network stack to flush first. */
  wifi_manager_save_sta_credentials_and_reboot(ssid, pass);
  return ESP_OK;
}

void wifi_web_config_start(void) {
  if (server != NULL) return;

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.stack_size = 4096;

  if (httpd_start(&server, &config) != ESP_OK) {
    server = NULL;
    ESP_LOGE(TAG, "failed to start config web server");
    return;
  }

  static const httpd_uri_t index_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = index_get_handler,
  };
  static const httpd_uri_t save_uri = {
      .uri = "/save",
      .method = HTTP_POST,
      .handler = save_post_handler,
  };
  httpd_register_uri_handler(server, &index_uri);
  httpd_register_uri_handler(server, &save_uri);

  ESP_LOGI(TAG, "config web page ready at http://%s/", wifi_manager_get_ip_str());
}

void wifi_web_config_stop(void) {
  if (server == NULL) return;
  httpd_stop(server);
  server = NULL;
}
