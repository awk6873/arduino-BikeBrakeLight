#include <SparkFunMPU9250-DMP.h>
#include <util/arduino_mpu9250_i2c.h>
#include "ArduinoLowPower.h"

#define MPU9250_ADDRESS     0x68  // Device address when ADO = 0; Use 0x69 when AD0 = 1
#define WHO_AM_I_MPU9250    0x75   // Should return 0x71; MPU9255 will return 0x73



const unsigned char reg[] = {
    0x75,
    0x19,
    0x1A,
    0x0C,
    0x6A,
    0x23,
    0x1B,
    0x1C,
    0x1D,
    0x1E,
    0x1F,
    0x20,
    0x72,
    0x43,
    0x3B,
    0x41,
    0x38,
    0x39,
    0x3A,
    0x69,
    0x6B,
    0x6C,
    0x37,
    0x77,
    0x24,
    0x6D,
    0x6E,
    0x70,
    0x49,
    0x25,
    0x26,
    0x27,
    0x28,
    0x29,
    0x2A,
    0x34,
    0x63,
    0x64,
    0x67,
    0x68
};


MPU9250_DMP imu;

void setup() 
{
  Serial.begin(115200);
  delay(5000);

  Wire.begin();
  Wire.setClock(400000);                            // 400 kbit/sec I2C speed

  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  delay(1000);

  if ((c == 0x71) || (c == 0x73)) // MPU9250=0x68; MPU9255=0x73
  {
    Serial.println(F("MPU9250 is online..."));
    Serial.println("");
  }
}

void loop() 
{
  unsigned char ii;
  unsigned char data;
  
  for (ii = 0; ii < 40; ii++) {
        Serial.print(reg[ii], HEX);
        Serial.print("\t");
        Serial.println(readByte(0x68, reg[ii]), BIN);
    }
  Serial.println();
  delay(10000);  
}

byte readByte(byte address, byte subAddress)
{
  byte data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (byte) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}
