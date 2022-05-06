#define __Queue_INIT
#include "common.h"


//------------------------------------------------------------------------------
TRecQ System::Queue[_SIZE_QUEUE_];
volatile u8 System::wr_queue=0, System::rd_queue=0;
TRecW System::Wait[_SIZE_W_];
TMemory System::MemoryState;
TError System::SysError;

//------------------------------------------------------------------------------
//__monitor - выполнение функции не может быть прервано ПРЕРЫВАНИЕМ
void System::push_queue(pulf3ul Func,u32 ulP1,u32 ulP2,u32 ulP3)
{
	__istate_t isate=__get_interrupt_state();
	__disable_interrupt();
	if(Func!=0)
	{
		System::Queue[wr_queue].dwEmpty=0;
		System::Queue[wr_queue].func=Func;
		System::Queue[wr_queue].par[0]=ulP1;
		System::Queue[wr_queue].par[1]=ulP2;
		System::Queue[wr_queue].par[2]=ulP3;
		if(++System::wr_queue>=_SIZE_QUEUE_)
			System::wr_queue=0;
		if(System::wr_queue==System::rd_queue){//фатальная ошибка!!!
			System::SysError.ErrQueue=1;
		}
	}
	__set_interrupt_state(isate);
}
void System::push_queue(pulf2ul Func,u32 ulP1,u32 ulP2)
{
	System::push_queue((pulf3ul)Func, ulP1, ulP2, 0);
}
void System::push_queue(pulf1ul Func,u32 ulP1)
{
	System::push_queue((pulf3ul)Func, ulP1, 0, 0);
}
void System::push_queue(pulfv Func)
{
	System::push_queue((pulf3ul)Func, 0, 0, 0);
}
//------------------------------------------------------------------------------
void System::push_queue(Object*	pObj,pulm3ul Metod,u32 ulP1,u32 ulP2,u32 ulP3)
{
	__istate_t isate=__get_interrupt_state();
	__disable_interrupt();
	if(Metod!=0)
	{
		System::Queue[wr_queue].pObj=pObj;
		System::Queue[wr_queue].metod=Metod;
		System::Queue[wr_queue].par[0]=ulP1;
		System::Queue[wr_queue].par[1]=ulP2;
		System::Queue[wr_queue].par[2]=ulP3;
		if(++System::wr_queue>=_SIZE_QUEUE_)
			System::wr_queue=0;
		if(System::wr_queue==System::rd_queue){//фатальная ошибка!!!
			System::SysError.ErrQueue=1;
		}
	}
	__set_interrupt_state(isate);
}
void System::push_queue(Object*	pObj,pulm2ul Metod,u32 ulP1,u32 ulP2)
{
	System::push_queue(pObj, (pulm3ul)Metod, ulP1, ulP2, 0);
}
void System::push_queue(Object*	pObj,pulm1ul Metod,u32 ulP1)
{
	System::push_queue(pObj, (pulm3ul)Metod, ulP1, 0, 0);
}
void System::push_queue(Object*	pObj,pulmv Metod)
{
	System::push_queue(pObj, (pulm3ul)Metod, 0, 0, 0);
}
//------------------------------------------------------------------------------
void run(void)
{
	for(;;)
	{
		if(System::rd_queue==System::wr_queue)	return;				//пусто
		TRecQ *pnt=&System::Queue[System::rd_queue];
		System::exec(pnt->pObj,pnt->metod,pnt->par[0],pnt->par[1],pnt->par[2]);
		if(++System::rd_queue>=_SIZE_QUEUE_) System::rd_queue=0;
	}
}
//------------------------------------------------------------------------------
void System::set_time_out(u32 ulTime,pulf3ul Func,u32 ulP1,u32 ulP2,u32 ulP3)
{
	for(u8 n=0;n<_SIZE_W_;n++)
	{
		if(System::Wait[n].time==0)
		{	//пустая
			System::Wait[n].time=ulTime;
			System::Wait[n].rec.dwEmpty=0;
			System::Wait[n].rec.func=Func;
			System::Wait[n].rec.par[0]=ulP1;
			System::Wait[n].rec.par[1]=ulP2;
			System::Wait[n].rec.par[2]=ulP3;
			return;
		}
	}
	//фатальная ошибка!!!
	System::SysError.ErrWait=1;
}

void System::set_time_out(u32 ulTime,pulf2ul Func, u32 ulP1,u32 ulP2)
{
	System::set_time_out(ulTime, (pulf3ul)Func, ulP1, ulP2, 0);
}
void System::set_time_out(u32 ulTime,pulf1ul Func, u32 ulP1)
{
	System::set_time_out(ulTime, (pulf3ul)Func, ulP1, 0, 0);
}
void System::set_time_out(u32 ulTime,pulfv Func)
{
	System::set_time_out(ulTime, (pulf3ul)Func, 0, 0, 0);
}
//------------------------------------------------------------------------------
void System::set_time_out(u32 ulTime,Object* pObj,pulm3ul Metod,u32 ulP1,u32 ulP2,u32 ulP3)
{
	for(u8 n=0;n<_SIZE_W_;n++)
	{
		if(System::Wait[n].time==0)
		{	//пустая
			System::Wait[n].time=ulTime;
			System::Wait[n].rec.pObj=pObj;
			System::Wait[n].rec.metod=Metod;
			System::Wait[n].rec.par[0]=ulP1;
			System::Wait[n].rec.par[1]=ulP2;
			System::Wait[n].rec.par[2]=ulP3;
			return;
		}
	}
	//фатальная ошибка!!!
	System::SysError.ErrWait=1;
}
void System::set_time_out(u32 ulTime,Object* pObj,pulm2ul Metod,u32 ulP1,u32 ulP2)
{
	System::set_time_out(ulTime, pObj, (pulm3ul)Metod, ulP1, ulP2, 0);
}
void System::set_time_out(u32 ulTime,Object* pObj,pulm1ul Metod,u32 ulP1)
{
	System::set_time_out(ulTime, pObj, (pulm3ul)Metod, ulP1, 0, 0);
}
void System::set_time_out(u32 ulTime,Object* pObj,pulmv Metod)
{
	System::set_time_out(ulTime, pObj, (pulm3ul)Metod, 0, 0, 0);
}
//------------------------------------------------------------------------------
void System::clr_time_out(void)
{
	for(u8 n=0;n<_SIZE_W_;n++)
	{
		System::Wait[n].time=0;	
	}
}
//------------------------------------------------------------------------------
void System::clr_time_out(pulf3ul Func)
{
	for(u8 n=0;n<_SIZE_W_;n++)
	{
		if((System::Wait[n].time!=0)&&(System::Wait[n].rec.func==Func))
			System::Wait[n].time=0;
	}
}
//------------------------------------------------------------------------------
void System::clr_time_out(pulf3ul Func, u8 u8_Slot)
{
	for(u8 n=0;n<_SIZE_W_;n++)
	{
		if((System::Wait[n].time!=0)&&(System::Wait[n].rec.func==Func)&&
			 (System::Wait[n].rec.par[0]==u8_Slot))
			System::Wait[n].time=0;
	}
}
//------------------------------------------------------------------------------
void System::clr_time_out(Object* pObj,pulm3ul Metod)
{
	for(u8 n=0;n<_SIZE_W_;n++)
	{
		if((System::Wait[n].time!=0)&&(System::Wait[n].rec.pObj==pObj)&&(System::Wait[n].rec.metod==Metod))
			System::Wait[n].time=0;
	}
}
//------------------------------------------------------------------------------//
void test_time_out(void)
{
	for(u8 n=0;n<_SIZE_W_;n++)
	{
		if(System::Wait[n].time!=0)
		{
			if(--System::Wait[n].time==0){
				if(System::Wait[n].rec.dwEmpty==0)
					System::push_queue(System::Wait[n].rec.func,System::Wait[n].rec.par[0],
														 System::Wait[n].rec.par[1],System::Wait[n].rec.par[2]);
				else
					System::push_queue(System::Wait[n].rec.pObj,System::Wait[n].rec.metod,System::Wait[n].rec.par[0],
														 System::Wait[n].rec.par[1],System::Wait[n].rec.par[2]);
			}
		}
	}
}
//------------------------------------------------------------------------------
/*
//------------------------------------------------------------------------------
//Поиск утечки памяти
typedef struct
{
	u32 link;
	u32 size;
	u32 pnt;
	u32 num;
}TMonHeap;

TMonHeap MonHeap[128];
//Конец блока поиска утечки памяти
//------------------------------------------------------------------------------
*/
extern "C++" void *operator new(size_t size)//throw(std::bad_alloc)
{
	u32 link = __get_LR();
	void* ptr = malloc(size);
	if(ptr==NULL){
		System::SysError.ErrCountAlloc++;
		if(System::SysError.ErrCountAlloc==7){
			System::SysError.ErrHeap = 1;
			AllocTrap();
		}
		return NULL;
	}
	System::MemoryState.CountNew++;
	System::MemoryState.MemReserved+=*((u32*)ptr-1)&0xFFFFFFFC;

//------------------------------------------------------------------------------
//	Место для отладки
	
/*
//------------------------------------------------------------------------------
//Поиск утечки памяти
	for(u16 n = 0; n < sizeof(MonHeap) / sizeof(TMonHeap); n++)
	{
		if(MonHeap[n].pnt == 0)
		{
			MonHeap[n].link = link;
			MonHeap[n].size = size;
			MonHeap[n].pnt = (u32)ptr;
			MonHeap[n].num = System::MemoryState.CountNew - 1;
			break;
		}
	}

	TMonHeap temp; // временная переменная для обмена элементов местами
  // Сортировка массива пузырьком
  for (int i = 0; i < 128 - 1; i++) {
      for (int j = 0; j < 128 - i - 1; j++) {
          if (MonHeap[j].num < MonHeap[j + 1].num) {
              // меняем элементы местами
              temp = MonHeap[j];
              MonHeap[j] = MonHeap[j + 1];
              MonHeap[j + 1] = temp;
          }
      }
  }
//Конец блока поиска утечки памяти
//------------------------------------------------------------------------------
*/

	return ptr;
}
u32 RegLR;
#pragma segment="HEAP"
extern "C++" void operator delete(void *ptr)
{
	if((ptr >= __segment_begin("HEAP")) && (ptr <= __segment_end("HEAP")))
	{	//указатель из "кучи"
		System::MemoryState.MemReserved-=*((u32*)ptr-1)&0xFFFFFFFC;
		System::MemoryState.CountDel++;
/*
//------------------------------------------------------------------------------
//Поиск утечки памяти
	for(u16 n = 0; n < sizeof(MonHeap) / sizeof(TMonHeap); n++)
	{
		if(MonHeap[n].pnt == (u32)ptr)
		{
			MonHeap[n].link = 0;
			MonHeap[n].size = 0;
			MonHeap[n].pnt = 0;
			MonHeap[n].num = 0;
			break;
		}
	}

	TMonHeap temp; // временная переменная для обмена элементов местами
 // Сортировка массива пузырьком
 for (int i = 0; i < 128 - 1; i++) {
     for (int j = 0; j < 128 - i - 1; j++) {
         if (MonHeap[j].num < MonHeap[j + 1].num) {
             // меняем элементы местами
             temp = MonHeap[j];
             MonHeap[j] = MonHeap[j + 1];
             MonHeap[j + 1] = temp;
         }
     }
 }
//Конец блока поиска утечки памяти
//------------------------------------------------------------------------------
*/
	}
}
bool CheckAlloc(u32* Mem, pulf3ul Func, u32 u32P1, u32 u32P2, u32 u32P3)
{
	if(Mem==NULL){
		System::set_time_out(1 msec,Func,u32P1,u32P2,u32P3);
		return 0;
	}
	return 1;
}

