#ifndef I2CTalonSensor_h
#define I2CTalonSensor_h

#include "I2CTalon.h"

class I2CTalonSensor : public Sensor {
protected:
	I2CTalon& talon;
	I2CTalonSensor(I2CTalon& talon);

public:
	// C++ Can't require static methods, but these two are needed
	// static void isPresent(I2CTalon& talon, uint8_t port)
	// constructor accepting (I2CTalon& talon, uint8_t port)
};

struct I2CTalonSensorFactory {
	bool (*isPresent)(I2CTalon& talon, uint8_t port);
	I2CTalonSensor* (*create)(I2CTalon& talon, uint8_t port);

	template <typename Sensor>
	static I2CTalonSensor* CreateSensor(I2CTalon& talon, uint8_t port) {
		// If you see errors here, your sensor is lacking a constructor accepting (SDI12Talon& talon, uint8_t port)
		return new Sensor(talon, port);
	}
	template <typename Sensor>
	static constexpr I2CTalonSensorFactory Create() {
		// If you see errors here, your sensor is lacking an isPresent(SDI12Talon& talon, uint8_t port)
		bool (*isPresent)(I2CTalon& talon, uint8_t port) = Sensor::isPresent;
		I2CTalonSensor* (*create)(I2CTalon& talon, uint8_t port) = CreateSensor<Sensor>;
		return {isPresent, create};
	}
};

#endif // I2CTalonSensor_h
