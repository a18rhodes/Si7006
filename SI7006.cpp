/**************************************************************************/
/*!
@file     SI7006.cpp
@author   Austin Rhodes
@license  BSD (see license.txt)

Driver for the SI7006 digital temperature humidity sensor.

@section  HISTORY

v1.0 - First release
v1.1 - Update begin function to accept alternate pins
*/
/**************************************************************************/
#ifdef __AVR
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#endif
#include <stdlib.h>
#include <math.h>

#include "Si7006.h"

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/
Si7006::Si7006(void) {
  // Si7006 object
}

boolean Si7006::begin(int SDA, int SCL)
{
  if (SDA == -1 || SCL == -1){
    Wire.begin();
  }else{
    Wire.begin(SDA, SCL);
  }
  Wire.beginTransmission(Si7006_ADDRESS);
  if(Wire.endTransmission()){
    return false;
  }

  _Si7006Initialised = true;
  return true;
}


boolean Si7006::getID(char (&deviceID)[8]){
  byte high, low;
  for(uint i = 0; i < sizeof(deviceID); i++){
    deviceID[i] = 0;
  }
  Wire.beginTransmission(Si7006_ADDRESS);
  Wire.write(Si7006_READ_ID_HIGH_0);
  Wire.write(Si7006_READ_ID_HIGH_1);
  if (Wire.endTransmission()) return false;
  delay(100);
  Wire.requestFrom(Si7006_ADDRESS, 4);
  deviceID[7] |= Wire.read();
  deviceID[6] |= Wire.read();
  deviceID[5] |= Wire.read();
  deviceID[4] |= Wire.read();
  
  Wire.beginTransmission(Si7006_ADDRESS);
  Wire.write(Si7006_READ_ID_LOW_0);
  Wire.write(Si7006_READ_ID_LOW_1);
  if (Wire.endTransmission()) return false;
  delay(100);
  Wire.requestFrom(Si7006_ADDRESS, 4);
  deviceID[3] |= Wire.read();
  deviceID[2] |= Wire.read();
  deviceID[1] |= Wire.read();
  deviceID[0] |= Wire.read();

  return true;

}

boolean Si7006::getTemperature(float &temperature, uint8_t readOld){
  byte high, low;
  unsigned int value;
  Wire.beginTransmission(Si7006_ADDRESS);
  if (readOld){
    Wire.write(Si7006_READ_OLD_TEMP);
  }else{
    Wire.write(Si7006_MEAS_TEMP_NO_MASTER_MODE);
  }
  if (Wire.endTransmission()) return false;
  delay(100);
  Wire.requestFrom(Si7006_ADDRESS, 2);
  high = Wire.read();
  low = Wire.read();
  value = (high << 8) | low;
  // A temperature measurement will always return XXXXXX00 in the LSB field.
  if (value & 0xFFFC) {
    temperature = (172.72 * (float)value) / 65536 - 46.85;
  } else {
    Serial.println("Error on temp");
    return 1;
  }
  return 0;
}

boolean Si7006::getHumidity(float &humidity){
  byte high, low;
  unsigned int value;
  Wire.beginTransmission(Si7006_ADDRESS);
  Wire.write(Si7006_MEAS_REL_HUMIDITY_NO_MASTER_MODE);
  if (Wire.endTransmission()) return false;
  delay(100);
  Wire.requestFrom(Si7006_ADDRESS, 2);
  high = Wire.read();
  low = Wire.read();
  value = (high << 8) | low;
  //From datasheet: A humidity measurement will always return XXXXXX10 in the LSB field.
  if (value & 0xFFFE) {
    humidity = ((125 * (float)value ) / 65536) - 6;
  } else {
    return false;
  }
}

boolean Si7006::getTempHumidity(float &humidity, float &temperature){
  getHumidity(humidity);
  //onto the temperature.
  getTemperature(temperature, 1);
}
