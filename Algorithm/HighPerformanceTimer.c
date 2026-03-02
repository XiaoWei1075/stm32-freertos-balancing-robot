#include "stm32f103xb.h"

extern uint32_t SystemCoreClock;

void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;	// 使能调试与追踪模块
	DWT->CYCCNT = 0;	// 清零周期计数器
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;	// 使能 CYCCNT
}

uint32_t get_high_performance_tick(void)
{
	return DWT->CYCCNT;
}

float delta_time_ms(uint32_t t1)
{
	uint32_t t2 = DWT->CYCCNT;
	uint32_t delta_cycles = t2 - t1;	// 无符号回绕
	return (float)delta_cycles * 1000.0f / (float)SystemCoreClock;
}
