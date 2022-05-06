#ifndef __types_H
#define __types_H

#pragma pack(1)
//using u8 = unsigned char;
//using u16 = unsigned short;
//using u32 = unsigned long;
//using u64 = unsigned long long;
//
//using s8 = signed char;
//using s16 = signed short;
//using s32 = signed long;
//using s64 = signed long long;

union u8bitMap
{
	u8 value_;
	struct
	{
		u8 bit0 : 1;
		u8 bit1 : 1;
		u8 bit2 : 1;
		u8 bit3 : 1;
		u8 bit4 : 1;
		u8 bit5 : 1;
		u8 bit6 : 1;
		u8 bit7 : 1;
	};
	u8bitMap(u8 value) {value_ = value; }
};
union u16bitMap
{
	u8 value_;
	struct
	{
		u8 bit0 : 1;
		u8 bit1 : 1;
		u8 bit2 : 1;
		u8 bit3 : 1;
		u8 bit4 : 1;
		u8 bit5 : 1;
		u8 bit6 : 1;
		u8 bit7 : 1;
		u8 bit8 : 1;
		u8 bit9 : 1;
		u8 bit10 : 1;
		u8 bit11 : 1;
		u8 bit12 : 1;
		u8 bit13 : 1;
		u8 bit14 : 1;
		u8 bit15 : 1;
	};
	u16bitMap(u8 value) {value_ = value; }
};
union u32bitMap
{
	u8 value_;
	struct
	{
		u8 bit0 : 1;
		u8 bit1 : 1;
		u8 bit2 : 1;
		u8 bit3 : 1;
		u8 bit4 : 1;
		u8 bit5 : 1;
		u8 bit6 : 1;
		u8 bit7 : 1;
		u8 bit8 : 1;
		u8 bit9 : 1;
		u8 bit10 : 1;
		u8 bit11 : 1;
		u8 bit12 : 1;
		u8 bit13 : 1;
		u8 bit14 : 1;
		u8 bit15 : 1;
		u8 bit16 : 1;
		u8 bit17 : 1;
		u8 bit18 : 1;
		u8 bit19 : 1;
		u8 bit20 : 1;
		u8 bit21 : 1;
		u8 bit22 : 1;
		u8 bit23 : 1;
		u8 bit24 : 1;
		u8 bit25 : 1;
		u8 bit26 : 1;
		u8 bit27 : 1;
		u8 bit28 : 1;
		u8 bit29 : 1;
		u8 bit30 : 1;
		u8 bit31 : 1;
	};
	u32bitMap(u8 value) {value_ = value; }
};

class Object;

typedef u32 (Object::*pulm3ul)(u32,u32,u32);
typedef u32 (Object::*pulm2ul)(u32,u32);
typedef u32 (Object::*pulm1ul)(u32);
typedef u32 (Object::*pulmv)(void);

typedef u32 (*pulf3ul)(u32,u32,u32);
typedef u32 (*pulf2ul)(u32,u32);
typedef u32 (*pulf1ul)(u32);
typedef u32 (*pulfv)(void);

#pragma pack()

#define	Bit(x)	1 << x

#endif

