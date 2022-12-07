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
	~AS5600(){};
public:
	uint16_t 	GetAngle(void);
	uint16_t 	GetRawAngle(void);
	uint8_t  	GetStatus(void);
	void 		test();

private:
	void 		WriteReg(uint8_t Reg, uint8_t Data);
	uint8_t 	ReadReg(uint8_t Reg);

private:
	// SENSOR ADDRESS
	uint8_t 	AS5600_ADDR = 0x36 << 1;
	// SENSOR CONFIGURATION REGISTERS
	uint8_t		ZMCO 		= 0x00;
	uint8_t 	ZPOS_H		= 0x01;
	uint8_t		ZPOS_L		= 0x02;
	uint8_t		MPOS_H		= 0x03;
	uint8_t		MPOS_L		= 0x04;
	uint8_t		MANG_H		= 0x05;
	uint8_t		MANG_L		= 0x06;
	uint8_t		CONF_H		= 0x07;
	uint8_t		CONF_L		= 0x08;
	// SENSOR OUTPUT REGISTERS
	uint8_t		RAWANG_H	= 0x0C;
	uint8_t 	RAWANG_L	= 0x0D;
	uint8_t 	ANGLE_H		= 0x0E;
	uint8_t 	ANGLE_L		= 0x0F;
	// SENSOR STATUS REGISTERS
	uint8_t     STATUS		= 0x0B;
	uint8_t		AGC			= 0x1A;
	uint8_t		MAGNITUDE_H = 0x1B;
	uint8_t		MAGNITUDE_L = 0x1C;
	// SENSOR BURN COMMANDS
	uint8_t		BURN		= 0xFF;
private:
	I2C_HandleTypeDef 	hi2c;
	UART_HandleTypeDef huart;

};

#endif /* INC_MAGNETIC_SENSOR_HPP_ */
