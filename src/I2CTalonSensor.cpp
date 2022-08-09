#include "I2CTalonSensor.h"

I2CTalonSensor::I2CTalonSensor(I2CTalon& talon) : talon(talon) {
	talonPort = talon.getTalonPort();
	sensorInterface = BusType::I2C;
}