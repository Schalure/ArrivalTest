#ifndef __Queue_H
#define __Queue_H

#ifdef __Queue_INIT
#define __Queue
#else
#define __Queue	extern
#endif

/*----------------------------------------------------------------------------*/
/*											Объявление типов и переменных													*/
/*----------------------------------------------------------------------------*/
#define _SIZE_QUEUE_	128
#define _SIZE_W_	32
/*
class Object;
typedef u32 (Object::*pulm3ul)(u32,u32,u32);
typedef u32 (ulf3ul)(u32,u32,u32);
typedef ulf3ul *pulf3ul;

typedef void (vfv)(void);
typedef vfv *pvfv;

typedef u32 (ulfv)(void);
typedef ulfv *pulfv;

typedef u32 (dwfv)(void);
typedef dwfv *pdwfv;
*/
//------------------------------------------------------------------------------
//	Структура очереди
struct TRecQ{
	union{
		Object*	pObj;
		u32 dwEmpty;
	};
	union{
		pulf3ul	func;
		pulm3ul	metod;
		u32 mf;
	};
	u32 par[3];
};
//------------------------------------------------------------------------------
//	Структура ожидания таймаута
typedef struct
{
	u32 time;
  TRecQ rec;
} TRecW;
//------------------------------------------------------------------------------
//	стуруктура системных ошибок
struct TError{
	u8 ErrHeap				:1;		//	переполнение кучи
	u8 ErrCountAlloc	:3;		//	количество ошибочных выделений памяти подряд
	u8 ErrWait				:1;		//	переполнение поля ожидания
	u8 ErrQueue				:1;		//	переполнение очереди
/*
	u8 ErrRStack			:1;		//0 - переполнение программного стека   01
	u8 ErrCStack			:1;		//1 - переполнение аппаратного стека    02
	u8 ErrEEPROM			:1;		//5 - ошибка EEPROM                     20
	u8 ErrHard				:1;		//6 - аппаратная ошибка                 40
	u8 ErrSoft				:1;		//7 - ошибка программного кода          80
	u8 ErrLink				:1;		//8 - ошибка связи (UDP)                01
	u8 ErrRAM					:1;		//9 - ошибка ОЗУ                        02
	u8 ErrCheck				:1;		//10 - ошибка в контрольном номере      04
	u8 ErrCheckTS			:1;		//20 - ошибка в контрольном номере ТС   08
*/
};
//------------------------------------------------------------------------------
//	Статистика выделенной памяти
struct TMemory{
	u32 CountNew;
	u32 CountDel;
	u32 MemReserved;
};
/*----------------------------------------------------------------------------*/
/*															Класс системы																	*/
/*----------------------------------------------------------------------------*/
class System{
public:
	//	Статистика выделенной памяти
	TMemory static MemoryState;
	//	Состояние системы
	TError static SysError;
	//	Переменные очереди
	TRecQ static Queue[_SIZE_QUEUE_];
	TRecW static Wait[_SIZE_W_];
	volatile u8 static wr_queue,rd_queue;
	//	Функции постановки в очередь
	void static push_queue(pulf3ul Func,u32 ulP1,u32 ulP2,u32 ulP3);
	void static push_queue(pulf2ul Func,u32 ulP1,u32 ulP2);
	void static push_queue(pulf1ul Func,u32 ulP1);
	void static push_queue(pulfv Func);
	void static push_queue(Object*	pObj,pulm3ul Metod,u32 ulP1,u32 ulP2,u32 ulP3);
	void static push_queue(Object*	pObj,pulm2ul Metod,u32 ulP1,u32 ulP2);
	void static push_queue(Object*	pObj,pulm1ul Metod,u32 ulP1);
	void static push_queue(Object*	pObj,pulmv Metod);
	//	Функции запуска
	void static exec(Object* pObj,pulm3ul func,u32 par1,u32 par2,u32 par3);
	void static __pass(pulf3ul func,u32 par1,u32 par2,u32 par3);
	void static __pass(Object* pObj,pulm3ul metod,u32 par1,u32 par2,u32 par3);
	u32 static __ret(Object* pObj,pulm3ul metod,u32 par1,u32 par2,u32 par3);
	u32 static __ret(Object* pObj,pulm2ul metod,u32 par1,u32 par2);
	u32 static __ret(Object* pObj,pulm1ul metod,u32 par1);
	u32 static __ret(Object* pObj,pulmv metod);
	u32 static __ret(pulf3ul func, u32 par1, u32 par2, u32 par3);
	u32 static __ret(pulf2ul func, u32 par1, u32 par2);
	u32 static __ret(pulf1ul func, u32 par1);
	u32 static __ret(pulfv func);
	//	Функции ожидания
	void static set_time_out(u32 ulTime,pulf3ul Func,u32 ulP1,u32 ulP2,u32 ulP3);
	void static set_time_out(u32 ulTime,pulf2ul Func,u32 ulP1,u32 ulP2);
	void static set_time_out(u32 ulTime,pulf1ul Func,u32 ulP1);
	void static set_time_out(u32 ulTime,pulfv Func);
	void static set_time_out(u32 ulTime,Object* pObj,pulm3ul Metod,u32 ulP1,u32 ulP2,u32 ulP3);
	void static set_time_out(u32 ulTime,Object* pObj,pulm2ul Metod,u32 ulP1,u32 ulP2);
	void static set_time_out(u32 ulTime,Object* pObj,pulm1ul Metod,u32 ulP1);
	void static set_time_out(u32 ulTime,Object* pObj,pulmv Metod);
	//	Сбросить ожидание
	void static clr_time_out(void);
	void static clr_time_out(pulf3ul Func);
	void static clr_time_out(Object* pObj,pulm3ul Metod);
	void static clr_time_out(pulf3ul Func, u8 u8_Slot);
};
//	Основная рабочая ф-я системы
void run(void);
//	Проверка ф-й на ожидание
void test_time_out(void);
//	Ловушки
void AllocTrap(void);
bool CheckAlloc(u32* Mem, pulf3ul Func, u32 u32P1, u32 u32P2, u32 u32P3);
#endif