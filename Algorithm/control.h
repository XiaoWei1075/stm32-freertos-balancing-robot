#ifndef __CONTROL_HEADER__
#define __CONTROL_HEADER__

extern SemaphoreHandle_t globalMutex;

extern EventGroupHandle_t eventGroup;
typedef enum EVENT
{
	EVENT_INTERRUPT_TRIGGERED = 0x1 << 0,
	EVENT_BMI160_INITIALIZED = 0x1 << 1,
	EVENT_MOTOR_ENABLED = 0x1 << 2,
	
} EVENT;

extern QueueHandle_t shellQueue;
typedef struct SHELLMSG
{
	const uint8_t* buffer;
	uint16_t len;
} SHELLMSG;

void imuTask(void* param);
void pidTask(void* param);
void shellTask(void* param);

#endif // __CONTROL_HEADER__
