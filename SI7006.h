/***************************************************************************
/*
/* Written by Austin Rhodes, based off Adafruit general Arduino Libraries
/*    and thingTronics source codes
/* https://github.com/automote/Si7006/
/* https://github.com/adafruit/Adafruit_TCS34725
/*
****************************************************************************/

#ifndef Si7006_h
#define Si7006_h

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Wire.h>
#define Si7006_ADDRESS                          0x40 // default address

// Si7006 register addresses
#define Si7006_MEAS_REL_HUMIDITY_MASTER_MODE    0xE5
#define Si7006_MEAS_REL_HUMIDITY_NO_MASTER_MODE 0xF5
#define Si7006_MEAS_TEMP_MASTER_MODE            0xE3
#define Si7006_MEAS_TEMP_NO_MASTER_MODE         0xF3
#define Si7006_READ_OLD_TEMP                    0xE0
#define Si7006_RESET                            0xFE
#define Si7006_WRITE_HUMIDITY_TEMP_CONTR        0xE6
#define Si7006_READ_HUMIDITY_TEMP_CONTR         0xE7
#define Si7006_WRITE_HEATER_CONTR               0x51
#define Si7006_READ_HEATER_CONTR                0x11
#define Si7006_READ_ID_LOW_0                    0xFA
#define Si7006_READ_ID_LOW_1                    0x0F
#define Si7006_READ_ID_HIGH_0                   0xFC
#define Si7006_READ_ID_HIGH_1                   0xC9
#define Si7006_FIRMWARE_0                       0x84
#define Si7006_FIRMWARE_1                       0xB8


class Si7006 {
	public:
		Si7006(void);
		boolean begin(void);
		boolean reset(void);
		boolean getTempControl(byte &res, boolean voltage, boolean heater);
		boolean setTempControl(byte res, boolean heater);
		boolean getHeaterControl(byte &heaterCurrent);
		boolean setHeaterControl(byte heaterCurrent);
		boolean getTemperature(float &temperature, boolean mode);
		boolean getHumidity(float &humidity, boolean mode);
    boolean getTempHumidity(float &humidity, float &temp, boolean mode);
    boolean read16(uint8_t reg, unsigned int &value);
    boolean read8(uint8_t reg, byte &value);
    boolean write8 (uint8_t reg, uint32_t value);

	private:
		boolean _Si7006Initialised;
};

#endif
