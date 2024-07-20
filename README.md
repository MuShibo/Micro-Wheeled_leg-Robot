# Micro-Wheeled_leg-Robot
> **The smallest two-wheeled-leg robot!**
>
> Vedio：[[自制]首款桌面级双轮腿机器人](https://www.bilibili.com/video/BV1io4y1q73L/?spm_id_from=333.999.0.0)
>

| Real Robot        | 3D Design        |
| ------------ | ------------ |
| <img src="4.Docs/Image/RobotReal.jpg" alt="Image 1" height="150"/> | <img src="4.Docs/Image/RobotRender.png" alt="Image 2" height="150"/> |

[中文文档](README_CN.md)

### Mechanical Structure Documentation

* The file "OriginalRobotModel.stp" is the robot model file.
* The "Parts-Manufactured" folder contains parts that need to be processed and produced by yourself, including 3D printing, CNC, and panel cutting.
* The "Parts-Purchased" folder contains parts that need to be purchased.

### PCB Documentation

* There are three PCBs that need to be fabricated.
* The main control board is based on the ESP32, with the brushless motor driver chip being the L6234PD013TR. Be cautious as there are many counterfeit chips available on Taobao.
* The encoder used is the AS5600 with an I2C interface.
* The IMU used is the MPU6050 module, which shares the same I2C interface with the left-side encoder.
* The circuit board provides both the schematic and PCB source files, and the IDE used is JLCPCB EDA.

### Source Code Usage

* Based on Arduino IDE, it is very simple.
* The brushless motor drive for the wheels is based on [simpleFOC](https://www.simplefoc.com/#simplefoc_library).
* The ESP32 itself has WiFi capabilities, and the webpage code is stored in Flash, with JSON data transmitted via the WebSocket communication protocol.

### Usage Instructions

* 1. Connect the battery's XH2.54 plug to the rear interface of the main control board and turn on the switch to power the robot.
* 2. After turning on the switch, a red light on the main board will indicate that the power is on.
* 3. Next, the wheels will start FOC motor initialization, with the wheels moving slightly and the legs starting to move.
* 4. If the battery is sufficient, the blue LED on the main control board will light up. If it does not light up, charging is needed.
* 5. After this process is complete, press the EN button on the main control board to restart, and you can then connect to the robot's WiFi network starting with WL. The password is the lowercase WiFi name.
* 6. Open a browser and go to the URL 192.168.1.11. The remote control interface is compatible with Android, iOS, Windows, Linux, macOS, etc., and it is recommended to use Chrome or Firefox.
* 7. Manually stabilize the robot, with the wheels slightly touching the ground. Click the "Robot go!" button on the web page, and the robot will stand up. You can then control the robot's movement using the joystick.

#### Open Source Contributors

* Mu Shibo
* Li Yufeng
