#define __SysTick_INIT

#include "common.h"

//------------------------------------------------------------------------------
//	Инициализация системного тика (int Time - от 1 до 10)
void Init_SysTick(int Time)
{
	//	Запись значения сброса
	SysTick->LOAD = SystemCoreClock * Time / 1000 - 1;
	//	Установки: Включение | Разрешение прерывания | Выбор частоты (внутренняя)
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk|
									SysTick_CTRL_TICKINT_Msk|
									SysTick_CTRL_CLKSOURCE_Msk;
}
//------------------------------------------------------------------------------

//	Прерывание системного таймера
extern "C" void SysTick_Handler(void)
{
	u32_TickCount++;
}
