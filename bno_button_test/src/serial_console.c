#include "serial_console.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "lwip/inet.h"

#include "state_machine.h"
#include "osc_config.h"

static const char *TAG = "console";

#define LINE_BUF_SIZE 128

/* Human-readable names, indexed the same as enum device_state_t, purely
 * for the "status" command's output. */
static const char *state_name(enum device_state_t state) {
  switch (state) {
    case DEVICE_STATE_GLOWSTICK_GREEN: return "GLOWSTICK_GREEN";
    case DEVICE_STATE_GLOWSTICK_PURPLE: return "GLOWSTICK_PURPLE";
    case DEVICE_STATE_GLOWSTICK_LIME: return "GLOWSTICK_LIME";
    case DEVICE_STATE_GLOWSTICK_BLUE: return "GLOWSTICK_BLUE";
    case DEVICE_STATE_GLOWSTICK_RED: return "GLOWSTICK_RED";
    case DEVICE_STATE_GLOWSTICK_WHITE: return "GLOWSTICK_WHITE";
    case DEVICE_STATE_GLOWSTICK_BLAST: return "GLOWSTICK_BLAST";
    case DEVICE_STATE_SWING_MODE: return "SWING_MODE";
    case DEVICE_STATE_OSC_SWING_MODE: return "OSC_SWING_MODE";
    default: return "UNKNOWN";
  }
}

static void print_help(void) {
  printf(
      "\r\n--- serial console: mode switching without the physical button ---\r\n"
      "  help            show this list\r\n"
      "  status          print current device state + OSC registration info\r\n"
      "  next            advance one state (same as a short button press)\r\n"
      "  state <n>       jump directly to state n (0-%d, see 'status' for names)\r\n"
      "  osc             shortcut for 'state %d' (DEVICE_STATE_OSC_SWING_MODE)\r\n"
      "  heap            print free/minimum-ever-free heap (diagnostic)\r\n"
      "--------------------------------------------------------------------\r\n",
      (int)DEVICE_STATE_OSC_SWING_MODE, (int)DEVICE_STATE_OSC_SWING_MODE);
}

static void print_heap(void) {
  printf("\r\nfree heap: %u bytes (DRAM: %u, min-ever DRAM: %u)\r\n",
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));
}

static void print_status(void) {
  const osc_config_t *cfg = osc_config_get();
  struct in_addr addr = {.s_addr = cfg->target_ip};
  printf("\r\nstate: %d (%s)\r\n", (int)device_state, state_name(device_state));
  printf("osc target: %s:%u  stream_enabled=%d  stream_period_ms=%u\r\n",
         cfg->target_ip ? inet_ntoa(addr) : "(none)", cfg->target_port, cfg->stream_enabled,
         cfg->stream_period_ms);
}

/* Parses and runs one command line (already stripped of the trailing
 * newline). Unknown input is ignored other than an error hint, since this
 * is a small debug aid, not a full shell. */
static void handle_line(char *line) {
  while (*line == ' ') line++;
  size_t len = strlen(line);
  while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' || line[len - 1] == ' ')) {
    line[--len] = '\0';
  }
  if (len == 0) return;

  if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
    print_help();
  } else if (strcmp(line, "status") == 0) {
    print_status();
  } else if (strcmp(line, "heap") == 0) {
    print_heap();
  } else if (strcmp(line, "next") == 0) {
    next_device_state();
    ESP_LOGI(TAG, "-> next_device_state(): now %d (%s)", (int)device_state, state_name(device_state));
  } else if (strcmp(line, "osc") == 0) {
    state_machine_force(DEVICE_STATE_OSC_SWING_MODE);
    ESP_LOGI(TAG, "-> forced state %d (%s)", (int)device_state, state_name(device_state));
  } else if (strncmp(line, "state ", 6) == 0) {
    const char *arg = line + 6;
    while (*arg == ' ') arg++;
    if (isdigit((unsigned char)*arg)) {
      const int requested = atoi(arg);
      state_machine_force((enum device_state_t)requested);
      ESP_LOGI(TAG, "-> forced state %d (%s)", (int)device_state, state_name(device_state));
    } else {
      printf("usage: state <n>\r\n");
    }
  } else {
    printf("unknown command \"%s\" (try \"help\")\r\n", line);
  }
}

static void serial_console_task(void *arg) {
  (void)arg;
  char line[LINE_BUF_SIZE];
  size_t line_len = 0;
  uint8_t rx_byte;

  ESP_LOGI(TAG, "serial console ready - type \"help\" and press enter");

  for (;;) {
    const int read = usb_serial_jtag_read_bytes(&rx_byte, 1, pdMS_TO_TICKS(50));
    if (read <= 0) continue;

    if (rx_byte == '\n' || rx_byte == '\r') {
      if (line_len > 0) {
        line[line_len] = '\0';
        handle_line(line);
        line_len = 0;
      }
      continue;
    }

    if (line_len < LINE_BUF_SIZE - 1) {
      line[line_len++] = (char)rx_byte;
    }
  }
}

void start_serial_console_task(void) {
  /* The secondary log console (CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG)
   * only wires up TX, so reading commands back over the same USB link needs
   * the USB-Serial/JTAG driver installed for RX too. If something else
   * already installed it, this returns ESP_ERR_INVALID_STATE, which is
   * fine - the driver is a singleton and usb_serial_jtag_read_bytes() below
   * still works against that existing instance. */
  usb_serial_jtag_driver_config_t usj_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  const esp_err_t err = usb_serial_jtag_driver_install(&usj_cfg);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "usb_serial_jtag_driver_install failed: %s", esp_err_to_name(err));
  }

  xTaskCreate(serial_console_task, "console", 4096, NULL, 3, NULL);
}
