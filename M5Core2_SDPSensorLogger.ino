#include <M5Core2.h>

#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>

#include "bsp_log.h"
#include "sdcard.h"
#include "record.h"
#include "SHT3x.h"
#include "board.h"

static const char *TAG = "main";

extern void sdcard_init(fs::SDFS &fs);


//SHT3x sht30(0x44, &Wire);
//Adafruit_BMP280 bmp(&Wire);


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

  record_sdp_start();
//  record_bmp_start();
}


void loop() {
  vTaskDelete(NULL);
//  bmp280_read(fileBMP);
  if (M5.Lcd.getCursorY() >= 240) {
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 3 * 8);  // skip the BMP logging
  }
  M5.update();
  vTaskDelay(pdMS_TO_TICKS(100));
}
