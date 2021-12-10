#include "esp32-hal-log.h"

#define BSP_LOGE(TAG, fmt, ...) do {log_e(fmt, ##__VA_ARGS__); bsp_log(ESP_LOG_ERROR, fmt, ##__VA_ARGS__);} while(0)
#define BSP_LOGW(TAG, fmt, ...) do {log_w(fmt, ##__VA_ARGS__); bsp_log(ESP_LOG_WARN, fmt, ##__VA_ARGS__);} while(0)
#define BSP_LOGI(TAG, fmt, ...) do {log_i(fmt, ##__VA_ARGS__); bsp_log(ESP_LOG_INFO, fmt, ##__VA_ARGS__);} while(0)


void bsp_log_init();
void bsp_log(esp_log_level_t level, const char *format, ...);
