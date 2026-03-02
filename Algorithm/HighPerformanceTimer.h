#ifndef __HIGHPERFORMANCETIMER_HEADER__
#define __HIGHPERFORMANCETIMER_HEADER__

void DWT_Init(void);	// 初始化DWT计数器

uint32_t get_high_performance_tick(void);	// 得到DWT计数值

float delta_time_ms(uint32_t t1);	// 计算DWT时间差

#endif // __HIGHPERFORMANCETIMER_HEADER__
