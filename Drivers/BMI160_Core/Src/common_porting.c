/*
 * common_porting.c
 *
 *  Created on: Mar 30, 2022
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit
 */

#include <string.h>
#include "i2c.h"
#include "common_porting.h"
#include "FreeRTOS.h"
#include "task.h"

void delay_ms(uint16_t period)
{
	vTaskDelay(period);
}

/*******************************************************************************
* Function Name  : I2C_Read
* Description    : Read data from I2C device registers
* Input          : I2C2 device_address, register address, data, data lenght
* Output         : None
* Return         : 0
*******************************************************************************/
int8_t SensorAPI_I2Cx_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	HAL_I2C_Master_Transmit(&I2C_HANDLE, BMI160_ADDR, &reg_addr, 1, I2CTIMEOUT);
	HAL_I2C_Master_Receive(&I2C_HANDLE, BMI160_ADDR, data, len, I2CTIMEOUT);
	return 0;
}

/*******************************************************************************
* Function Name  : I2C_Write
* Description    : Write data into I2C device registers
* Input          : I2C2 device_address, register address, data, data lenght
* Output         : None
* Return         : 0
*******************************************************************************/
int8_t SensorAPI_I2Cx_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t* GTXBuffer = (uint8_t*)pvPortMalloc(len + 1);
	if (GTXBuffer == NULL)
		return -1;
	GTXBuffer[0] = reg_addr;
	memcpy(&GTXBuffer[1], data, len);
	HAL_I2C_Master_Transmit(&I2C_HANDLE, BMI160_ADDR, GTXBuffer, len + 1, I2CTIMEOUT);
	vPortFree(GTXBuffer);
	return 0;
}
