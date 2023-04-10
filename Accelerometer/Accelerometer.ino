#include <Wire.h>

// Addresses
#define I2C 0x1D
#define STATUS 0x00
#define OUT_Z_MSB 0x05
#define OUT_Z_LSB 0x06
#define CTRL_REG1 0x2A

// Constants
#define ACC_FACTOR 4096;

void setup()
{
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);

  // Standby
  WriteRegister(CTRL_REG1, 0x00); 

  // Set Values
  WriteRegister(CTRL_REG1, 0xFC);

  // Start
  WriteRegister(CTRL_REG1, 0xFD);
}

void loop()
{
  // put your main code here, to run repeatedly:
  uint8_t status = ReadRegister(STATUS);

  if(status & 0x04)
  {
    int16_t zAccRaw = FetchZAcceleration();
    float zAcc = (zAccRaw * 9.7953) / ACC_FACTOR;
    Serial.print(zAcc);
    Serial.print("\n");
  }
}

void WriteRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(I2C);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t ReadRegister(uint8_t reg)
{
  Wire.beginTransmission(I2C);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(I2C, 1);
  return Wire.read();
}

uint16_t FetchZAcceleration()
{
  uint8_t ZMSB = ReadRegister(OUT_Z_MSB);
  uint8_t ZLSB = ReadRegister(OUT_Z_LSB);
  return (int16_t)((ZMSB << 8) | (ZLSB)) >> 2;

}
