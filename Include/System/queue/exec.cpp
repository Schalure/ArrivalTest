#include "common.h"
//---------------------------------------------------------------
TRecQ bank;
//------------------------------------------------------------------------------
void System::__pass(pulf3ul func,u32 par1,u32 par2,u32 par3)
{
	bank.dwEmpty=0;
	bank.func=func;
	bank.par[0]=par1;
	bank.par[1]=par2;
	bank.par[2]=par3;
}
//------------------------------------------------------------------------------
void System::__pass(Object* pObj,pulm3ul metod,u32 par1,u32 par2,u32 par3)
{
	bank.pObj=pObj;
	bank.metod=metod;
	bank.par[0]=par1;
	bank.par[1]=par2;
	bank.par[2]=par3;
}
//------------------------------------------------------------------------------
u32 System::__ret(pulf3ul func, u32 par1, u32 par2, u32 par3)
{
	System::__pass(func, par1, par2, par3);
	return 1;
}
u32 System::__ret(pulf2ul func, u32 par1, u32 par2)
{
	System::__pass((pulf3ul)func, par1, par2, 0);
	return 1;
}
u32 System::__ret(pulf1ul func, u32 par1)
{
	System::__pass((pulf3ul)func, par1, 0, 0);
	return 1;
}
u32 System::__ret(pulfv func)
{
	System::__pass((pulf3ul)func, 0, 0, 0);
	return 1;
}
//------------------------------------------------------------------------------
u32 System::__ret(Object* pObj, pulm3ul metod, u32 par1, u32 par2, u32 par3)
{
	System::__pass(pObj, metod, par1, par2, par3);
	return 1;
}
u32 System::__ret(Object* pObj, pulm2ul metod, u32 par1, u32 par2)
{
	System::__pass(pObj, (pulm3ul)metod, par1, par2, 0);
	return 1;
}
u32 System::__ret(Object* pObj, pulm1ul metod, u32 par1)
{
	System::__pass(pObj, (pulm3ul)metod, par1, 0, 0);
	return 1;
}
u32 System::__ret(Object* pObj, pulmv metod)
{
	System::__pass(pObj, (pulm3ul)metod, 0, 0, 0);
	return 1;
}
//------------------------------------------------------------------------------
void System::exec (Object* pObj,pulm3ul metod,u32 par1,u32 par2,u32 par3)
{
	u32 next = 0;
	System::__pass(pObj,metod,par1,par2,par3);
	do
	{
		if(bank.dwEmpty==0)
			next=(u32)bank.func(bank.par[0],bank.par[1],bank.par[2]);
		else
			next=(u32)(bank.pObj->*bank.metod)(bank.par[0],bank.par[1],bank.par[2]);
	}
	while(next!=0);
}
//------------------------------------------------------------------------------
void AllocTrap(void)
{
	while(1);
}

