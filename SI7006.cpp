/**************************************************************************/
/*!
@file     Adafruit_TCS34725.cpp
@author   KTOWN (Adafruit Industries)
@license  BSD (see license.txt)

Driver for the TCS34725 digital color sensors.

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

@section  HISTORY

v1.0 - First release
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

boolean Si7006::begin(void)
{
  Wire.begin();
  Wire.beginTransmission(Si7006_ADDRESS);
  if(Wire.endTransmission()){
    return false;
  }

  _Si7006Initialised = true;
  return true;
}


boolean Si7006::getTempHumidity(float &humidity, float &temperature){
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
  //onto the temperature.
  Wire.beginTransmission(Si7006_ADDRESS);
  Wire.write(Si7006_READ_OLD_TEMP);
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
  }
}
