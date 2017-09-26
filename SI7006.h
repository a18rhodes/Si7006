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


class Si7006 {
	public:
		Si7006(void);
		boolean begin(void);
    boolean getTempHumidity(float &humidity, float &temperature);

	private:
		boolean _Si7006Initialised;
};

#endif
