#include <M5Core2.h>

#include "bsp_log.h"
#include "sdcard.h"
#include "record.h"
#include "board.h"

static const char *TAG = "main";

extern void sdcard_init(fs::SDFS &fs);


void setup() {
  bsp_log_init();  
  bool LCDEnable = true;
  bool SDEnable = true;
  bool SerialEnable = true;
  bool I2CEnablePortA = false;  // BMP280 sensor
  M5.begin(LCDEnable, SDEnable, SerialEnable, I2CEnablePortA);
  M5.Lcd.clear();
  M5.Lcd.setTextSize(3);
  M5.Lcd.print("\n");  // reserve space for BMP logging

  Wire.begin(14, 13);
  Wire.setClock(400000);

  board_init();
  sdcard_init(SD);
//  sdcard_listdir(sdcard_mount_point, 0);
//  sdcard_print_content_prev("LOG.LOG");

//  record_bmp_start();
  record_sdp_start();

}


void loop() {
  vTaskDelete(NULL);
}
