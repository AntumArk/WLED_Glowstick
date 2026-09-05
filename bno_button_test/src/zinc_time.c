#include "zinc_time.h"
#include "esp_timer.h"

uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}
