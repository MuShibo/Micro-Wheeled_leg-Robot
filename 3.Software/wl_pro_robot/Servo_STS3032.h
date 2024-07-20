// -----------------------------------------------------------------------------
// Copyright (c) 2024 Mu Shibo
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// -----------------------------------------------------------------------------


//飞特STS3032舵机同步控制指令
//调用方式为sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

#ifndef _SERVO_STS3032_H
#define _SERVO_STS3032_H

	#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
	#else
	#include "WProgram.h"
	#endif

//数据结构定义
typedef	char s8;
typedef	unsigned char u8;	
typedef	unsigned short u16;	
typedef	short s16;
typedef	unsigned long u32;	
typedef	long s32;

//内存表定义
#define SMS_STS_ACC 41
#define INST_SYNC_WRITE 0x83

//飞特串行舵机通信层协议程序
class SCS{
public:
	SCS();
	SCS(u8 End);
	SCS(u8 End, u8 Level);
	void syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen);//同步写指令
public:
	u8 Level;//舵机返回等级
	u8 End;//处理器大小端结构
	u8 Error;//舵机状态
	u8 syncReadRxPacketIndex;
	u8 syncReadRxPacketLen;
	u8 *syncReadRxPacket;
	u8 *syncReadRxBuff;
	u16 syncReadRxBuffLen;
	u16 syncReadRxBuffMax;
	u32 syncTimeOut;
protected:
	virtual int writeSCS(unsigned char *nDat, int nLen) = 0;
	virtual int writeSCS(unsigned char bDat) = 0;
	virtual void rFlushSCS() = 0;
	virtual void wFlushSCS() = 0;
protected:
	void Host2SCS(u8 *DataL, u8* DataH, u16 Data);//1个16位数拆分为2个8位数
	int	Ack(u8 ID);//返回应答
	int checkHead();//帧头检测
};

//飞特串行舵机硬件接口层程序
class SCSerial : public SCS
{
public:
	SCSerial();
	SCSerial(u8 End);
	SCSerial(u8 End, u8 Level);
protected:
	int writeSCS(unsigned char *nDat, int nLen);//输出nLen字节
	int writeSCS(unsigned char bDat);//输出1字节
	void rFlushSCS();//
	void wFlushSCS();//
public:
	unsigned long IOTimeOut;//输入输出超时
	HardwareSerial *pSerial;//串口指针
	int Err;
public:
	virtual int getErr(){  return Err;  }
};

//飞特SMS/STS系列串行舵机应用层程序
class SMS_STS : public SCSerial
{
public:
	virtual void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);//同步写多个舵机位置指令
};

#endif
