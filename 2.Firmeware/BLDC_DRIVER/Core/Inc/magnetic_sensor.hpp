/*
 * magnetic_sensor.hpp
 *
 *  Created on: Nov 24, 2022
 *      Author: hemingshan
 */

#ifndef INC_MAGNETIC_SENSOR_HPP_
#define INC_MAGNETIC_SENSOR_HPP_
#include "main.h"
#include "string.h"
#include "stdio.h"

class AS5600{
public:
	AS5600(I2C_HandleTypeDef &hi2c_, UART_HandleTypeDef &huart_);
	~AS5600();
public:
	uint16_t 	GetAngle(void);
	uint16_t 	GetRawAngle(void);
	uint8_t  	GetStatus(void);
	void 		test();

private:
	void 		WriteReg(uint8_t Reg, uint8_t Data);
	uint8_t 	ReadReg(uint8_t Reg);

private:
	uint8_t 	AS5600_ADDR = 0x36 << 1;
	uint8_t		RAWANG_H	= 0x0C;
	uint8_t 	RAWANG_L	= 0x0D;
	uint8_t 	ANGLE_H		= 0x0E;
	uint8_t 	ANGLE_L		= 0x0F;

private:
	I2C_HandleTypeDef 	hi2c;
	UART_HandleTypeDef huart;

};

#endif /* INC_MAGNETIC_SENSOR_HPP_ */
