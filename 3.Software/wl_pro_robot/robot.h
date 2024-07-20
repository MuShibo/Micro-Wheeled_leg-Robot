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

#pragma once

#include <WiFi.h>
#include <ArduinoJson.h>

typedef struct {
  int height = 38;
  int roll;
  int linear;
  int angular;
  int dir;
  int dir_last;
  int joyy;
  int joyy_last;
  int joyx;
  int joyx_last;
  bool go;
}Wrobot;

extern Wrobot wrobot;

// 机器人运动状态枚举
typedef enum {
	FORWARD = 0, 	
	BACK,        	       		
	RIGHT,       	
	LEFT,        	
	STOP,
  JUMP,         		
} QR_State_t; 


	
// 机器人模式枚举类型
typedef enum {
  BASIC = 0,
} Robot_Mode_t; 

class RobotProtocol
{
	public:
		RobotProtocol(uint8_t len);
		~RobotProtocol();
		void spinOnce(void);
		void parseBasic(StaticJsonDocument<300> &doc);
    private:
        uint8_t *_now_buf;
		uint8_t *_old_buf;
		uint8_t _len;
		void UART_WriteBuf(void);
		int checkBufRefresh(void);
};
