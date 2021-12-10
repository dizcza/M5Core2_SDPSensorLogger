#include <M5Core2.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "bsp_log.h"
#include "sdcard.h"
#include "board.h"

static xSemaphoreHandle logSemaphore;

static const char *LOG_LEVEL_CHR = "NEWIDV";


void bsp_log_init() {
  logSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(logSemaphore);
}


void bsp_log(esp_log_level_t level, const char *format, ...) {
  static char log_buffer[512];

  if (logSemaphore == NULL) return;
  xSemaphoreTake(logSemaphore, portMAX_DELAY);
  va_list args;
  va_start( args, format );
  size_t offset = snprintf(log_buffer, sizeof(log_buffer), "%c (%lld) ",
                           LOG_LEVEL_CHR[level], esp_timer_get_time());
  offset += vsnprintf(log_buffer + offset, sizeof(log_buffer) - offset, format, args);
  uint16_t color = LIGHTGREY;
  switch (level) {
    case ESP_LOG_ERROR:
      color = RED;
      break;
    case ESP_LOG_WARN:
      color = ORANGE;
      break;
    case ESP_LOG_INFO:
      color = GREEN;
      break;
  }
  M5.Lcd.setTextColor(color);
  M5.Lcd.setTextSize(1);
  if (M5.Lcd.getCursorY() >= 240) {
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 3 * 8);  // skip the BMP logging
  }
  M5.Lcd.println(log_buffer);
  log_buffer[offset++] = '\n';
  log_buffer[offset++] = '\0';
  sdcard_log(log_buffer);
  va_end( args );
  xSemaphoreGive(logSemaphore);
}

