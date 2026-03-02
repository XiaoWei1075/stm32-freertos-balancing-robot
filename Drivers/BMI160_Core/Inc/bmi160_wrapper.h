/*
 * bmi160_wrapper.h
 *
 *  Created on: Mar 30, 2022
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit
 */

#ifndef BMI160_WRAPPER_H_
#define BMI160_WRAPPER_H_

#include "common_porting.h"

typedef struct {
	float BMI160_Ax_f32, BMI160_Ay_f32, BMI160_Az_f32;
	float BMI160_Gx_f32, BMI160_Gy_f32, BMI160_Gz_f32;
	float Gx_bias, Gy_bias, Gz_bias;
} BMI160_t;

extern BMI160_t imu_t;	// BMI160 Handler

int8_t BMI160_init(void);
int8_t bmi160ReadAccelGyro(BMI160_t *DataStruct, uint8_t ZeroVelocityUpdate);
int8_t bmi160Gyro_Zero_Calibration(BMI160_t *DataStruct);
int8_t start_foc(void);

#endif /* BMI160_WRAPPER_H_ */
