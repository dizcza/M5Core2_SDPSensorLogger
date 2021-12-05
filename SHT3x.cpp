#include "SHT3x.h"


SHT3x::SHT3x(uint8_t i2c_addr, TwoWire *theWire)
{
  this->i2c_addr = i2c_addr;
  this->wire = theWire;
}


bool SHT3x::readTemperatureHumidity(float *temperature, float *humidity)
{
  unsigned int data[6];

  // Start I2C Transmission
  this->wire->beginTransmission(i2c_addr);
  // Send measurement command
  this->wire->write(0x2C);
  this->wire->write(0x06);
  // Stop I2C transmission
  if (this->wire->endTransmission()!=0) {
    return false;  
  }

  delay(200);

  // Request 6 bytes of data
  this->wire->requestFrom(i2c_addr, (uint8_t) 6);

  // Read 6 bytes of data
  // cTemp msb, cTemp lsb, cTemp crc, humidity msb, humidity lsb, humidity crc
  for (int i=0;i<6;i++) {
    data[i]=this->wire->read();
  };

  delay(50);
  
  if (this->wire->available()!=0) {
    return false;
  }

  // Convert the data
  if (temperature != NULL) *temperature = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
  if (humidity != NULL) *humidity = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);

  return true;
}
