#ifndef __SENSOR_SHT3x_H
#define __SENSOR_SHT3x_H

#include <stdint.h>
#include "Arduino.h"
#include "Wire.h"


class SHT3x {
public:
  SHT3x(uint8_t i2c_addr=0x44, TwoWire *theWire = &Wire);
  bool readTemperatureHumidity(float *temperature, float *humidity);

private:
  uint8_t i2c_addr;
  TwoWire *wire;
};


#endif  /* __SENSOR_SHT3x_H */
