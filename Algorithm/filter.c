#include "filter.h"

/**************************************************************************
Function: Simple Kalman filter
Input   : acceleration、angular velocity
Output  : none
函数功能：获取x轴角度简易卡尔曼滤波
入口参数：加速度获取的角度、角速度
返回  值：x轴角度
**************************************************************************/
float Kalman_Filter_x(float Accel, float Gyro, float dt)		
{
	static float angle=0.f;
	const float Q_angle=0.001; // 过程噪声的协方差
	const float Q_gyro=0.003;	//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	const float R_angle=0.5;		// 测量噪声的协方差 既测量偏差
	const char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	
	angle	+= K_0 * Angle_err;	//后验估计
	Q_bias	+= K_1 * Angle_err;	//后验估计
	return angle;
}
/**************************************************************************
Function: First order complementary filtering
Input   : acceleration、angular velocity
Output  : none
函数功能：一阶互补滤波
入口参数：加速度获取的角度、角速度
返回  值：x轴角度
**************************************************************************/
float Complementary_Filter_x(float angle_m, float gyro_m, float dt)
{
	static float angle=0.f;
	const float K1 = 0.05f;
	angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	return angle;
}
/**************************************************************************
Function: Simple Kalman filter
Input   : acceleration、angular velocity
Output  : none
函数功能：获取y轴角度简易卡尔曼滤波
入口参数：加速度获取的角度、角速度
返回  值：y轴角度
**************************************************************************/
float Kalman_Filter_y(float Accel, float Gyro, float dt)		
{
	static float angle=0.f;
	const float Q_angle=0.001; // 过程噪声的协方差
	const float Q_gyro=0.003;	//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	const float R_angle=0.5;		// 测量噪声的协方差 既测量偏差
	const char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	//后验估计
	Q_bias	+= K_1 * Angle_err;	//后验估计
	return angle;
}
/**************************************************************************
Function: First order complementary filtering
Input   : acceleration、angular velocity
Output  : none
函数功能：一阶互补滤波
入口参数：加速度获取的角度、角速度
返回  值：y轴角度
**************************************************************************/
float Complementary_Filter_y(float angle_m, float gyro_m, float dt)
{
	static float angle=0.f;
	const float K1 = 0.05f;
	angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	return angle;
}
/**************************************************************************
函数功能：Yaw角一阶互补滤波
入口参数：Yaw             Yaw角
          encoder_z_rate  编码器计算的Z轴角速度 (rad/s 或 deg/s)
          gyro_z_rate     陀螺仪Z轴角速度	    (rad/s 或 deg/s)
返回  值：Yaw角 (rad 或 deg，与输入角速度单位一致)
**************************************************************************/
float Complementary_Filter_Yaw(float Yaw, float encoder_z_rate, float gyro_z_rate, float dt)
{
	const float K1 = 0.05f;	// 编码器低频权重
	float yaw_encoder = Yaw + encoder_z_rate * dt;	// 编码器角速度积分得到角度
	float yaw_gyro = Yaw + gyro_z_rate * dt;	// 陀螺角速度积分得到角度
	Yaw = K1 * yaw_encoder + (1.0f - K1) * yaw_gyro;
	return Yaw;
}
