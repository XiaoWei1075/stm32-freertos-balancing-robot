#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>

#include "tim.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "bmi160_wrapper.h"
#include "filter.h"
#include "control.h"
#include "HighPerformanceTimer.h"

#define PI ((float)3.1415926535897932384626433832795)
//车轮直径(m)
#define DIAMETER (0.068f)
//轮间距(m)
#define WHEEL_SPACING (0.175f)
//编码器的分辨率，轮子每转一圈编码器产生的脉冲数
#define RESOLUTION (1320)	//11*30*4
//车以1m/s行驶1ms，编码器读数
#define ENCODER1msPerUNITSPEED (RESOLUTION*0.001f/PI/DIAMETER)
//编码器的电机死区（PWM测量值，行星减速器充分耦合）
#define ZERO_ZONE_ENCODER 186
//认定小车为静止的持续时间阈值（计次）
#define STATIC_THRESHOLD 200
//PWM输出限幅
#define MAX_PWM 7000

//是否启用电机死区补偿（实验不建议启用）
#undef ZERO_ZONE_CORRECTION

SemaphoreHandle_t globalMutex;

EventGroupHandle_t eventGroup;

QueueHandle_t shellQueue;

BMI160_t imu_t;	// BMI160 Handler
static volatile float g_Pitch, /*g_Roll, */g_Yaw;	// 欧拉角
static volatile int g_speed;	// 直线速度
static volatile float TargetVelocity;	// 目标直线速度
static volatile float TargetGyroZ;	// 目标转向角速度

inline bool protect(float pitch);
inline int PWM_Limit(float in);
inline float PWM_zero_zone_correction(float PWM);

void stopMotor(void);
void startMotor(void);

float PI_Velocity(int current_velocity, bool unBalance);
float get_balance_PWM(float angle, float balance, float gyro_balance);
float PID_Steering(float current_yaw, float current_gyro_z, bool unBalance);

void print_parameters(void);

static void DMP(int encoder_A, int encoder_B, float dt_ms)
{
	const float dt = dt_ms / 1000.f;
	const float Accel_Y = imu_t.BMI160_Ay_f32;	//Y轴加速度计
	const float Accel_Z = imu_t.BMI160_Az_f32;	//Z轴加速度计
	const float Accel_Angle_x = atan2f(Accel_Y, Accel_Z) * 180.f / PI;	//计算俯仰角，转换单位为度
	const float Pitch = -Kalman_Filter_x(Accel_Angle_x, imu_t.BMI160_Gx_f32, dt);
	// 将编码器数值转换为相应的速度，再转换为角速度，单位：度每秒
	const float encoder_gyro_z = (encoder_B - encoder_A) / ENCODER1msPerUNITSPEED / dt_ms / WHEEL_SPACING * 180.f / PI;
	// 编码器角速度与陀螺仪角速度的符号应该相同，数值应该相近
	const float Yaw   = Complementary_Filter_Yaw(g_Yaw, encoder_gyro_z, imu_t.BMI160_Gz_f32, dt);	//计算偏航角
	xSemaphoreTake(globalMutex, portMAX_DELAY);
	g_Pitch = Pitch;
	g_Yaw   = Yaw;
	g_speed = (encoder_A + encoder_B);
	xSemaphoreGive(globalMutex);
	return;
}

void imuTask(void* param)
{
	uint32_t lastDWTTick = get_high_performance_tick();
	uint32_t static_duration_cnt = 0U;
	while (true)
	{
		xEventGroupWaitBits(eventGroup, EVENT_INTERRUPT_TRIGGERED | EVENT_BMI160_INITIALIZED, pdFALSE, pdTRUE, portMAX_DELAY);
		xEventGroupClearBits(eventGroup, EVENT_INTERRUPT_TRIGGERED);
		const int encoder_A = -Read_Encoder(2);	// 读取编码器A数值
		const int encoder_B = -Read_Encoder(1);	// 读取编码器B数值
		bool ZeroVelocity = (encoder_A == 0 && encoder_B == 0 && fabsf(g_Pitch) < 3.f && fabsf(TargetVelocity) < 1e-3f);
		ZeroVelocity = (ZeroVelocity && fabsf(TargetGyroZ) < 1e-3f && fabsf(imu_t.BMI160_Gx_f32) < 1.f && fabsf(imu_t.BMI160_Gz_f32) < 1.f);
		if (ZeroVelocity)
			++static_duration_cnt;
		else
			static_duration_cnt = 0U;
		const bool ZeroVelocityUpdate = (static_duration_cnt > STATIC_THRESHOLD);
		bmi160ReadAccelGyro(&imu_t, ZeroVelocityUpdate);	// 函数内部对globalMutex上锁解锁，保护imu_t
		float dt_ms = delta_time_ms(lastDWTTick);	// 计算循环时间差
		lastDWTTick = get_high_performance_tick();
		if (dt_ms > 100)
			continue;
		DMP(encoder_A, encoder_B, dt_ms);
	}
}

static float vc_to_ta_stand = 0.2f;
static float vc_to_ta_move = 0.3f;
void pidTask(void* param)
{
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (true)
	{
		xEventGroupWaitBits(eventGroup, EVENT_MOTOR_ENABLED, pdFALSE, pdTRUE, portMAX_DELAY);
		vTaskDelayUntil(&lastWakeTime, 5);
		xSemaphoreTake(globalMutex, portMAX_DELAY);
		const int speed = g_speed;
		const float Pitch = g_Pitch;
		const float Yaw = g_Yaw;
		const float gyroX = -imu_t.BMI160_Gx_f32;
		const float gyroZ = imu_t.BMI160_Gz_f32;
		xSemaphoreGive(globalMutex);
		const bool unBalance = protect(Pitch);
		const float speed_control = PI_Velocity(speed, unBalance); // 速度环输出
		float target_angle_offset;
		if (fabsf(TargetVelocity) < 1e-3f)
			target_angle_offset = speed_control * vc_to_ta_stand; // 直立状态，速度环输出缩放为角度偏移量
		else
			target_angle_offset = speed_control * vc_to_ta_move; // 移动状态，速度环输出缩放为角度偏移量
		const float balance_pwm = get_balance_PWM(Pitch, target_angle_offset, gyroX); // 直立环计算
		float steering_pwm = PID_Steering(Yaw, gyroZ, unBalance);	// 转向环输出
		// PWM 优先权保护
		float remainder = (float)MAX_PWM - fabsf(balance_pwm);
		if (remainder < 0.f)
			remainder = 0.f;
		if (steering_pwm > remainder)
			steering_pwm = remainder;
		else if (steering_pwm < -remainder)
			steering_pwm = -remainder;
		const int PWM1 = PWM_Limit(PWM_zero_zone_correction(balance_pwm + steering_pwm));
		const int PWM2 = PWM_Limit(PWM_zero_zone_correction(balance_pwm - steering_pwm));
		Set_PWM(PWM1, PWM2);	// 闭环控制-根据PWM值驱动电机
	}
}

bool protect(float pitch)
{
	if (fabsf(pitch) > 45)
	{
		stopMotor();
		return true;
	}
	else
		return false;
}

int PWM_Limit(float in)
{
	int out;
	if (in > (float)MAX_PWM)
		out = MAX_PWM;
	else if (in < -(float)MAX_PWM)
		out = -MAX_PWM;
	else
		out = roundf(in);
	return out;
}

float PWM_zero_zone_correction(float PWM)
{
#ifdef ZERO_ZONE_CORRECTION
	if (PWM > 0)
		PWM += ZERO_ZONE_ENCODER;
	else if (PWM < 0)
		PWM -= ZERO_ZONE_ENCODER;
#endif // ZERO_ZONE_CORRECTION
	return PWM;
}

void stopMotor(void)
{
	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);	// disable STBY
	xEventGroupClearBitsFromISR(eventGroup, EVENT_MOTOR_ENABLED);
}

void startMotor(void)
{
	xSemaphoreTake(globalMutex, portMAX_DELAY);
	g_Yaw = 0.f;
	g_speed = 0;
	xSemaphoreGive(globalMutex);
	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);	// enable STBY
	xEventGroupSetBitsFromISR(eventGroup, EVENT_MOTOR_ENABLED, NULL);
}

static float Ve_Kp = 2.087f, Ve_Ki = 0.015f;
float PI_Velocity(int current_velocity, bool unBalance)
{
	const float Kp = Ve_Kp;	// 比例增益
	const float Ki = Ve_Ki;	// 积分增益
	static float Encoder, Encoder_Integral;

	// 计算速度偏差
	float Encoder_Least = current_velocity - TargetVelocity;

	// 低通滤波
	Encoder = Encoder * 0.7f + Encoder_Least * 0.3f;

	if (unBalance)
		Encoder_Integral = 0.f;
	else
	{
		// 积分限幅
		Encoder_Integral += Encoder;
		if(Encoder_Integral > 1000)
			Encoder_Integral = 1000;
		else if(Encoder_Integral < -1000)
			Encoder_Integral = -1000; 
	}

	// PI控制
	float Velocity = Encoder * Kp + Encoder_Integral * Ki;

	return Velocity;
}

static float balance_Kp = 565.4866776f, balance_Kd = 30.1327412f;
float get_balance_PWM(float angle, float balance, float gyro_balance)
{
	const float Kp = balance_Kp;	// 比例增益
	const float Kd = balance_Kd;	// 微分增益
	float balance_pwm = Kp * (angle - balance) + Kd * gyro_balance;	// 通过 PD 控制计算平衡 PWM
	return balance_pwm;
}

static float turn_Kp = 17.45329f, turn_Kd = 1.5f;
static float yaw_Kp = 350.0f, yaw_Kd = 7.0f;
float PID_Steering(float current_yaw, float current_gyro_z, bool unBalance)
{
	const float TURN_KP = turn_Kp;
	const float TURN_KD = turn_Kd;
	const float YAW_KP = yaw_Kp;
	const float YAW_KD = yaw_Kd;

	static float last_error = 0.f;
	static bool is_turning_mode = false;
	if (unBalance)
	{
		is_turning_mode = false;
		last_error = 0.f;
		return 0.f;
	}

	float steering_pwm = 0.f;

	// 设定死区，过滤遥控器抖动
	if (fabsf(TargetGyroZ) > 1.f)
	{
		// 模式 1：主动转向 (角速度环)
		float error = TargetGyroZ - current_gyro_z;
		if (!is_turning_mode)
		{
			is_turning_mode = true;
			last_error = error;
		}

		float derivative = error - last_error;
		last_error = error;

		// PD控制：主要靠P跟随速度，D抑制超调
		steering_pwm = TURN_KP * error + TURN_KD * derivative;
	}
	else
	{
		// 模式 2：直线保持 (Yaw角度环)
		if (is_turning_mode)
		{
			is_turning_mode = false;
			current_yaw = 0.f;
			xSemaphoreTake(globalMutex, portMAX_DELAY);
			g_Yaw = 0.f;
			xSemaphoreGive(globalMutex);
		}

		// 计算角度偏差
		float yaw_error = 0.f - current_yaw;

		// 限制偏差大小，防止累积误差过大导致疯转
		if(yaw_error > 20.0f)
			yaw_error = 20.0f;
		else if(yaw_error < -20.0f)
			yaw_error = -20.0f;
		// P控制拉回角度 + D控制(GyroZ)抵抗外界旋转干扰
		// 这里的 D项 (current_gyro_z) 非常重要，它就是物理阻尼
		steering_pwm = YAW_KP * yaw_error - YAW_KD * current_gyro_z;
	}

	return steering_pwm;
}

void shellTask(void* param)
{
	while (true)
	{
		SHELLMSG shellMsg;
		xQueueReceive(shellQueue, &shellMsg, portMAX_DELAY);
		uint16_t i = 0;
		while (true)
		{
			for (; i < shellMsg.len && isspace(shellMsg.buffer[i]); ++i);
			if (i >= shellMsg.len)
				break;
			const char order = (char)shellMsg.buffer[i];
			if (order == ';')
			{
				++i;
				continue;
			}
			float data = 0.f;
			bool dot = false, negative = false;
			float pt = 10.f;
			for (++i; i < shellMsg.len; ++i)
			{
				const char res = (char)shellMsg.buffer[i];
				/*检测到指令结束*/
				if (res == ';')
				{
					++i;
					break;
				}
				/*提取浮点数data*/
				if (res == '.')
					dot = true;
				else if (res == '-')
					negative = true;
				else if (!isdigit(res))
					continue;
				else
				{
					if (!dot)
						data = 10.f * data + res - '0';
					else
					{
						data += (res - '0') / pt;
						pt *= 10.f;
					}
				}
			}
			/*接收完成，执行命令*/
			if (negative)
				data = -data;
			switch (order)
			{
			case '0':
				Ve_Kp = data;
				break;
			case '1':
				Ve_Ki = data;
				break;
			case '2':
				balance_Kp = data;
				break;
			case '3':
				balance_Kd = data;
				break;
			case '4':
				turn_Kp = data;
				break;
			case '5':
				turn_Kd = data;
				break;
			case '6':
				yaw_Kp = data;
				break;
			case '7':
				yaw_Kd = data;
				break;
			case '8':
				vc_to_ta_stand = data;
				break;
			case '9':
				vc_to_ta_move = data;
				break;
			case 'p':
				print_parameters();
				break;
			case 's':
				stopMotor();
				break;
			case 't':
				reset_all_timers();
				startMotor();
				break;
			case 'v':
				TargetVelocity = data;
				break;
			case 'g':
				TargetGyroZ = data;
				break;
			case 'z':
				TargetVelocity = 0.f;
				TargetGyroZ = 0.f;
				break;
			default:
				break;
			}
		}
	}
}

void print_parameters(void)
{
	static char string_buffer[512];
	if (huart3.gState != HAL_UART_STATE_READY)
		return;
	uint32_t ulNotificationValue = 0U;
	xTaskNotifyWait(0U, 0U, &ulNotificationValue, 0);
	float vcc = *(float*)(&ulNotificationValue);
	int i = sprintf(string_buffer, "pitch=%.2f, yaw=%.2f\n", g_Pitch, g_Yaw);
	i += sprintf(string_buffer + i, "speed=%d\n", g_speed);
	i += sprintf(string_buffer + i, "TargetVelocity=%.2f, TargetGyroZ=%.2f\n", TargetVelocity, TargetGyroZ);
	i += sprintf(string_buffer + i, "Ve_Kp=%.3f, Ve_Ki=%.3f\n", Ve_Kp, Ve_Ki);
	i += sprintf(string_buffer + i, "balance_Kp=%.3f, balance_Kd=%.3f\n", balance_Kp, balance_Kd);
	i += sprintf(string_buffer + i, "turn_Kp=%.3f, turn_Kd=%.3f\n", turn_Kp, turn_Kd);
	i += sprintf(string_buffer + i, "yaw_Kp=%.3f, yaw_Kd=%.3f\n", yaw_Kp, yaw_Kd);
	i += sprintf(string_buffer + i, "2stand=%.3f, 2move=%.3f\n", vc_to_ta_stand, vc_to_ta_move);
	i += sprintf(string_buffer + i, "vcc=%.3f\n", vcc);
	HAL_UART_Transmit_DMA(&huart3, (const uint8_t*)string_buffer, i);
}
