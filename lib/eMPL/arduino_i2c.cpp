#include "arduino_i2c.h"
#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>

int arduino_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	I2Cdev::writeBytes(slave_addr, reg_addr, length, data);

	return 0;
}

int arduino_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	I2Cdev::readBytes(slave_addr, reg_addr, length, data);

	return 0;
}
