#ifndef __SysTick_H
#define __SysTick_H

#ifdef __SysTick_INIT
#define __SysTick
#else
#define __SysTick	extern
#endif


//	Флаг времени
//Счетчик секунд
__SysTick	u32 u32_TickCount, u32_TickCountOld;

#define msec 	*1
#define sec 	*1000
#define minute	*60000UL

void Init_SysTick(int Time);

#endif