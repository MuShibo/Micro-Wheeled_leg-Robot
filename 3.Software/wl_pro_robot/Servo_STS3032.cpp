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

#include "Servo_STS3032.h"
#include <stddef.h>

//飞特串行舵机通信层协议程序
SCS::SCS()
{
	Level = 1;//除广播指令所有指令返回应答
	Error = 0;
}
SCS::SCS(u8 End)
{
	Level = 1;
	this->End = End;
	Error = 0;
}
SCS::SCS(u8 End, u8 Level)
{
	this->Level = Level;
	this->End = End;
	Error = 0;
}
//1个16位数拆分为2个8位数
//DataL为低位，DataH为高位
void SCS::Host2SCS(u8 *DataL, u8* DataH, u16 Data)
{
	if(End){
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}
//同步写指令
//舵机ID[]数组，IDN数组长度，MemAddr内存表地址，写入数据，写入长度
void SCS::syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	u8 mesLen = ((nLen+1)*IDN+4);
	u8 Sum = 0;
	u8 bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	writeSCS(bBuf, 7);

	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
	u8 i, j;
	for(i=0; i<IDN; i++){
		writeSCS(ID[i]);
		writeSCS(nDat+i*nLen, nLen);
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];
		}
	}
	writeSCS(~Sum);
	wFlushSCS();
}

//飞特串行舵机硬件接口层程序
SCSerial::SCSerial()
{
	IOTimeOut = 10;
	pSerial = NULL;
}
SCSerial::SCSerial(u8 End):SCS(End)
{
	IOTimeOut = 10;
	pSerial = NULL;
}
SCSerial::SCSerial(u8 End, u8 Level):SCS(End, Level)
{
	IOTimeOut = 10;
	pSerial = NULL;
}
int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial->write(nDat, nLen);
}
int SCSerial::writeSCS(unsigned char bDat)
{
	return pSerial->write(&bDat, 1);
}
void SCSerial::rFlushSCS()
{
	while(pSerial->read()!=-1);
}
void SCSerial::wFlushSCS()
{
}

//飞特SMS_STS系列串行舵机应用层程序
void SMS_STS::SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
    u8 offbuf[7*IDN];
    for(u8 i = 0; i<IDN; i++){
		if(Position[i]<0){
			Position[i] = -Position[i];
			Position[i] |= (1<<15);
		}
		u16 V;
		if(Speed){
			V = Speed[i];
		}else{
			V = 0;
		}
		if(ACC){
			offbuf[i*7] = ACC[i];
		}else{
			offbuf[i*7] = 0;
		}
        Host2SCS(offbuf+i*7+1, offbuf+i*7+2, Position[i]);
        Host2SCS(offbuf+i*7+3, offbuf+i*7+4, 0);
        Host2SCS(offbuf+i*7+5, offbuf+i*7+6, V);
    }
    syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
}

