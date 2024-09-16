# Micro-Wheeled_leg-Robot
> **The smallest two-wheeled-leg robot!**
>
> Video：[[自制]首款桌面级双轮腿机器人](https://www.bilibili.com/video/BV1io4y1q73L/?spm_id_from=333.999.0.0)
>

| Real Robot        | 3D Design        |
| ------------ | ------------ |
| <img src="4.Docs/Image/RobotReal.jpg" alt="Image 1" height="200"/> | <img src="4.Docs/Image/RobotRender.png" alt="Image 2" height="200"/> |

[中文文档](README_CN.md)

### Mechanical Structure Documentation

* The file "OriginalRobotModel.stp" is the robot model file.
* The "Parts-Manufactured" folder contains parts that need to be processed and produced by yourself, including 3D printing, CNC, and panel cutting.
* The "Parts-Purchased" folder contains parts that need to be purchased.

### PCB Documentation

* There are four PCBs that need to be fabricated; the circuit boards provide both the schematic and PCB source files, and the IDE used is [LCEDA](https://lceda.cn/).
* The main control board is based on the ESP32, with the brushless motor driver chip being the L6234PD013TR. 
* The encoder chip used is the AS5600, communicating with the main control board via the I2C interface.
* The IMU used is the MPU6050 module, which shares the same I2C interface with the right-side encoder.
* The servo debugging board unifies the two serial lines into one signal line, completing the task through time-division multiplexing to send and receive data.
* In addition, you will need three GH1.25 4PIN double-ended cables, with a recommended length of 15cm. These need to be purchased separately.

| Wire Connection | 
| ------------ |
| <img src="4.Docs/Image/Connection.png" alt="Image 3" height="500"/> |

### Source Code Usage

* Based on [Arduino IDE](https://www.arduino.cc/), it is very easy to learn and use.
* The brushless motor drive for the wheels is based on [simpleFOC](https://www.simplefoc.com/#simplefoc_library).
* The left bus servo ID is 1, and the right is 2; the calibration for the legs to fully squat to the mechanical limit position is 2048; configuration is done using [FEETECH Debug Software](https://gitee.com/ftservo/fddebug).
* The [ESP32](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf) itself has WiFi capabilities, with the webpage code stored in flash, transmitting JSON data via the WebSocket communication protocol.
* Please use the WebSocket library located in [3.Software/libraries](3.Software/libraries). The recommended esp32 version is 2.0.3.
* There are two WiFi models, AP mode and STA mode. AP mode uses the device as a wireless hotspot, and STA mode uses the device as a client to connect to an existing wireless network.

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
