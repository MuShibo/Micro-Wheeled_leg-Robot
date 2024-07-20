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

#include "wifi.h"

// 配置AP（热点）模式相关参数
const char *AP_SSID = "WLROBOT"; 
const char *AP_PSW = "12345678";    

IPAddress AP_IP(192, 168, 1, 11); 
IPAddress AP_GATEWAY(192, 168, 1, 11); 
IPAddress AP_SUBNET(255, 255, 255, 0); 


// 配置STA模式相关参数
char *sta_ssid = "MUJITECH";
char *sta_password = "mujitech";



void WiFi_SetAP(void)
{
	WiFi.mode(WIFI_AP); 
	WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET); 
	WiFi.softAP(AP_SSID, AP_PSW); 
	// Serial.println();
	// Serial.print("AP IP address = ");
	// Serial.println(WiFi.softAPIP());
}


// ESP-01S AP模式接入WiFi网络
void set_sta_wifi()
{

  WiFi.begin(sta_ssid, sta_password);
  // 等待连接
  while(WiFi.status()!=WL_CONNECTED)
  {
    //Serial.print(".");
    delay(500);
  }
  // 打印ESP-01S的IP地址
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}
