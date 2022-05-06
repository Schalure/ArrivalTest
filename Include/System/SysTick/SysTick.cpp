#define __SysTick_INIT

#include "common.h"

//------------------------------------------------------------------------------
//	������������� ���������� ���� (int Time - �� 1 �� 10)
void Init_SysTick(int Time)
{
	//	������ �������� ������
	SysTick->LOAD = SystemCoreClock * Time / 1000 - 1;
	//	���������: ��������� | ���������� ���������� | ����� ������� (����������)
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk|
									SysTick_CTRL_TICKINT_Msk|
									SysTick_CTRL_CLKSOURCE_Msk;
}
//------------------------------------------------------------------------------

//	���������� ���������� �������
extern "C" void SysTick_Handler(void)
{
	u32_TickCount++;
}
