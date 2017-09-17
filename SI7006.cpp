/***************************************************************************
/*
/* Written by Austin Rhodes, based off Adafruit general Arduino Libraries
/*    and thingTronics source codes
/* https://github.com/automote/Si7006/
/* https://github.com/adafruit/Adafruit_TCS34725
/*
****************************************************************************/

#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
#include <stdlib.h>
#include <math.h>

#include "Si7006.h"

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/


boolean Si7006::write8 (uint8_t reg, uint32_t value)
{
  Wire.beginTransmission(Si7006_ADDRESS);
  #if ARDUINO >= 100
  Wire.write(reg);
  Wire.write(value & 0xFF);
  #else
  Wire.send(reg);
  Wire.send(value & 0xFF);
  #endif
  Wire.endTransmission();
  return true;
}

boolean Si7006::read8(uint8_t reg, byte &value)
{
  Wire.beginTransmission(Si7006_ADDRESS);
  #if ARDUINO >= 100
  Wire.write(reg);
  #else
  Wire.send(reg);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(Si7006_ADDRESS, 1);
  #if ARDUINO >= 100
  value = Wire.read();
  #else
  value = Wire.receive();
  #endif

  return true;
}

boolean Si7006::read16(uint8_t reg, unsigned int &value)
{
  uint16_t x; uint16_t t;

  Wire.beginTransmission(Si7006_ADDRESS);
  #if ARDUINO >= 100
  Wire.write(reg);
  #else
  Wire.send(reg);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(Si7006_ADDRESS, 2);
  #if ARDUINO >= 100
  t = Wire.read();
  x = Wire.read();
  #else
  t = Wire.receive();
  x = Wire.receive();
  #endif
  x <<= 8;
  x |= t;
  value = x;

  return true;
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/
Si7006::Si7006(void) {
	// Si7006 object
}

boolean Si7006::begin(void)
{
  Wire.begin();

  _Si7006Initialised = true;

  return true;
}

boolean Si7006::reset(void) {
	// SW Reset the sensor
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

	// In reset we write default value to USER Register
	byte res = 0x00;
	boolean heater = false;
	if(setTempControl(res, heater)) {
		delay(15);
		return(true);
	}

	return(false);
}

boolean Si7006::getTempControl(byte &res, boolean voltage, boolean heater) {
	// Gets the contents RH/Temp User Register of the sensor
	// res uses D7 and D0 bit
	// If res = 0, RH is set to 12 bit & temp 14 bit resolution (default)
	// If res = 1, RH is set to 8 bit & temp 12 bit resolution
	// If res = 2, RH is set to 10 bit & temp 13 bit resolution
	// If res = 4, RH is set to 11 bit & temp 11 bit resolution
	//----------------------------------------------------------
	// If voltage = false(0), VDD OK (default)
	// If voltage = true(1), VDD LOW
	//----------------------------------------------------------
	// If heater = false(0), On-chip Heater is disabled (default)
	// If heater = true(1), On-chip Heater is disabled
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

	byte control, humidity;

	// Reading the control byte
	if(read8(Si7006_READ_HUMIDITY_TEMP_CONTR,control)) {
		// Extract resolution
		// extracting D7 control bit into D1 res; D0 control bit into D0 res
		res = ((control & 0x80) >> 6) | (control & 0x01);

		// Extract voltage
		voltage = (control & 0x40) ? true : false;

		// Extract heater
		heater = (control & 0x04) ? true : false;

		// return if successful
		return(true);
	}
	return(false);
}

boolean Si7006::setTempControl(byte res, boolean heater) {
	// Sets the contents RH/Temp User Register of the sensor
	// Gets the contents RH/Temp User Register of the sensor
	// res uses D7 and D0 bit
	// If res = 0, RH is set to 12 bit & temp 14 bit resolution (default)
	// If res = 1, RH is set to 8 bit & temp 12 bit resolution
	// If res = 2, RH is set to 10 bit & temp 13 bit resolution
	// If res = 4, RH is set to 11 bit & temp 11 bit resolution
	//----------------------------------------------------------
	// If heater = false(0), On-chip Heater is disabled (default)
	// If heater = true(1), On-chip Heater is disabled
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

	byte control = 0x00;

	// sanity check for gain
	if (res > 4) {
		res = 0x00;
	}

	// control byte logic
	control |= (res & 0x02) << 6 | (res & 0x01);

	if(heater) {
		control |= 0x04;
	}

	return(write8(Si7006_WRITE_HUMIDITY_TEMP_CONTR,control));
}

boolean Si7006::getHeaterControl(byte &heaterCurrent) {
	// Gets the Heater current of the On-chip Heater
	// If heaterCurrent = 0, Heater current is 3.09mA (default)
	// If heaterCurrent = 15, Heater current is 94.20mA
	// heaterCurrent is in multiples of 3.09mA
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

	// Reading the status byte
	if(read8(Si7006_READ_HEATER_CONTR,heaterCurrent)) {
		// Extract heater current
		heaterCurrent |= heaterCurrent & 0x0F;

		// return if successful
		return(true);
	}
	return(false);
}

boolean Si7006::setHeaterControl(byte heaterCurrent){
	// Sets the Heater current of the On-chip Heater
	// If heaterCurrent = 0, Heater current is 3.09mA (default)
	// If heaterCurrent = 15, Heater current is 94.20mA
	// heaterCurrent is in multiples of 3.09mA
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

	// sanity check
	if(heaterCurrent >= 15) {
		heaterCurrent = 0x15;
	}
	return(write8(Si7006_WRITE_HEATER_CONTR,heaterCurrent));
}

boolean Si7006::getTemperature(float &temperature, boolean mode) {
	// Gets the Temperature data from the sensor
	// If mode = true(1), Hold Master Mode is used
	// If mode = false(0), No Hold Master Mode is used
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

  if(!_Si7006Initialised){
    begin();
  }

	boolean success = false;
	unsigned int tempTemperature;

	if(mode) {
		if(!read16(Si7006_MEAS_TEMP_MASTER_MODE,tempTemperature))
			return(false);
	}
	else {
		if(!read16(Si7006_MEAS_TEMP_NO_MASTER_MODE,tempTemperature))
			return(false);
	}

	// Check if temperature is correct by ANDing with 0xFFFC
	if(tempTemperature & 0xFFFC) {
		temperature = (175.72 * (float)tempTemperature)/65536 - 46.85;
		return(true);
	}

	return(false);
}

boolean Si7006::getHumidity(float &humidity, boolean mode) {
	// Gets the Humidity data from the sensor
	// If mode = true(1), Hold Master Mode is used
	// If mode = false(0), No Hold Master Mode is used
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

  if(!_Si7006Initialised){
    begin();
  }

	boolean success = false;
	unsigned int tempHumidity;

	if(mode) {
		if(!read16(Si7006_MEAS_REL_HUMIDITY_MASTER_MODE,tempHumidity))
			return(false);
	}
	else {
		if(!read16(Si7006_MEAS_REL_HUMIDITY_NO_MASTER_MODE,tempHumidity))
			return(false);
	}

	// Check if humidity is correct by ANDing with 0xFFFE
	if(tempHumidity & 0xFFFE) {
		humidity = (125 * (float)tempHumidity)/65536 - 6;
		return(true);
	}

	return(false);
}

boolean Si7006::getTempHumidity(float &humidity, float &temp, boolean mode) {
	// Gets the Humidity data from the sensor
	// If mode = true(1), Hold Master Mode is used
	// If mode = false(0), No Hold Master Mode is used
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

  if(!_Si7006Initialised){
    begin();
  }

	boolean success = false;
	unsigned int tempHumidity;

	if(mode) {
		if(!read16(Si7006_MEAS_REL_HUMIDITY_MASTER_MODE,tempHumidity))
			return(false);
	}
	else {
		if(!read16(Si7006_MEAS_REL_HUMIDITY_NO_MASTER_MODE,tempHumidity))
			return(false);
	}
  if(!read16(Si7006_READ_OLD_TEMP, tempHumidity))
    return(false);

	// Check if humidity is correct by ANDing with 0xFFFE
	if(tempHumidity & 0xFFFE) {
		humidity = (125 * (float)tempHumidity)/65536 - 6;
		return(true);
	}

	return(false);
}
