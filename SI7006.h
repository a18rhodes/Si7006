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
#define Si7006_READ_ID_LOW_0					0xFA
#define Si7006_READ_ID_LOW_1					0x0F
#define Si7006_READ_ID_HIGH_0					0xFC
#define Si7006_READ_ID_HIGH_1					0xC9


class Si7006 {
	public:
		Si7006(void);
		boolean begin(int SDA = -1, int SCL = -1);
		boolean getTemperature(float &temperature, uint8_t readOld);
		boolean getHumidity(float &humidity);
    	boolean getTempHumidity(float &humidity, float &temperature);
		boolean getID(char (&deviceID)[8]);

	private:
		boolean _Si7006Initialised;
};

#endif
