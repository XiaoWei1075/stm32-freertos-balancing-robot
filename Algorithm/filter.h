#ifndef __FILTER_HEADER__
#define __FILTER_HEADER__

float Kalman_Filter_x(float Accel, float Gyro, float dt);		
float Complementary_Filter_x(float angle_m, float gyro_m, float dt);
float Kalman_Filter_y(float Accel, float Gyro, float dt);		
float Complementary_Filter_y(float angle_m, float gyro_m, float dt);

float Complementary_Filter_Yaw(float Yaw, float encoder_z_rate, float gyro_z_rate, float dt);

#endif	// __FILTER_HEADER__
