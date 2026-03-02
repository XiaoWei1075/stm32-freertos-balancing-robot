/*
 * common_porting.h
 *
 *  Created on: Mar 30, 2022
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit
 */

#ifndef COMMON_PORTING_H_
#define COMMON_PORTING_H_

#include <stdint.h>

#define I2C_HANDLE	(hi2c1)
#define BMI160_ADDR 0x69<<1
#define BUS_TIMEOUT 1000

#define I2CTIMEOUT 100

void delay_ms(uint16_t period);

int8_t SensorAPI_I2Cx_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t SensorAPI_I2Cx_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

#endif /* COMMON_PORTING_H_ */

