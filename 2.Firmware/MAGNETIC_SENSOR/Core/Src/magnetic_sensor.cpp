/*
 * magnetic_sensor.cpp
 *
 *  Created on: Nov 24, 2022
 *      Author: hemingshan
 */
#include "magnetic_sensor.hpp"

AS5600::AS5600(I2C_HandleTypeDef &hi2c_, UART_HandleTypeDef &huart_)
{
	/* I2C1 Initialization*/
	hi2c = hi2c_;

	hi2c.Instance = I2C1;
	hi2c.Init.ClockSpeed = 100000;
	hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c.Init.OwnAddress1 = 0;
	hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.OwnAddress2 = 0;
	hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c) != HAL_OK)
	{
		Error_Handler();
	}

	/* UART1 Initialization*/
	huart = huart_;

	huart.Instance = USART1;
	huart.Init.BaudRate = 115200;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart) != HAL_OK)
	{
		Error_Handler();
	}
}

void AS5600::WriteReg(uint8_t Reg, uint8_t Data)
{
	HAL_I2C_Master_Transmit(&hi2c,(AS5600_ADDR),&Data,1,10);
	HAL_I2C_Mem_Write(&hi2c,(AS5600_ADDR),Reg,1,&Data,1,100);
}

uint8_t AS5600::ReadReg(uint8_t Reg)
{
	uint8_t DataRead = 0;
	HAL_I2C_Mem_Read(&hi2c,(AS5600_ADDR),Reg,1,&DataRead,1,100);

	return DataRead;
}
uint16_t AS5600::GetAngle(void)
{
	uint8_t buf[12];
//	Data[0] = ReadReg(ANGLE_L);
//	Data[1] = (ReadReg(ANGLE_H) << 8);
	int Data = (ReadReg(ANGLE_L)|(ReadReg(ANGLE_H) << 8));
	sprintf((char*)buf,"%d\r\n",(ReadReg(ANGLE_L)|(ReadReg(ANGLE_H) << 8)));
	sprintf((char*)buf,"%d\r\n",Data);
	HAL_UART_Transmit(&huart, buf,  strlen((char*)buf), HAL_MAX_DELAY);
	HAL_Delay(100);
//	return Data;
	return 1;
}

uint8_t AS5600::GetStatus(void)
{
#if DEBUG
	uint8_t buf[12];
	sprintf((char*)buf,"%x\r\n",(ReadReg(STATUS) & 0x38));
	if ((ReadReg(STATUS) & 0x38) == 0x20)
	{
		strcpy((char*)buf, "Magnet!\r\n");
		HAL_UART_Transmit(&huart, buf, strlen((char*)buf), HAL_MAX_DELAY);
		HAL_Delay(100);
	}
	else{
		strcpy((char*)buf, "NO Magnet!\r\n");
		HAL_UART_Transmit(&huart, buf, 2, HAL_MAX_DELAY);
		HAL_Delay(100);
	}
#endif
	return ReadReg(STATUS) & 0x38;
}

void AS5600::test()
{
	/*
	 * Return: void
	 * Parameters: void
	 * Description: This function is used to test the UART and I2C protocol
	 */
	uint8_t buf[12];
	strcpy((char*)buf, "UART: OK\r\n");
	HAL_UART_Transmit(&huart, buf, strlen((char*)buf), HAL_MAX_DELAY);

	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(&hi2c, AS5600_ADDR, buf, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		strcpy((char*)buf, "I2C: Err\r\n");
		HAL_UART_Transmit(&huart, buf, strlen((char*)buf), HAL_MAX_DELAY);
	}
	else
	{
		strcpy((char*)buf, "I2C: OK\r\n");
		HAL_UART_Transmit(&huart, buf, strlen((char*)buf), HAL_MAX_DELAY);
	}
}
