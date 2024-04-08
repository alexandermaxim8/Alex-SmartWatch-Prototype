# Low-Cost Smartwatch Prototype
## About
Like a smartphone, the development of watches also rapidly becoming more sophisticated. The needs of easiness of time-tracking, is also heavily added with other features, making human activities more instantaneous. This prototype is made as a learning media, and could be also proposed as simpler and cheaper way to develop a gadget by each own. So if this project is continued further, people also can have their own customized gadget easily.

## Components and Architecture
![image](https://github.com/alexandermaxim8/Alex-SmartWatch-Prototype/assets/143409662/45fa1257-05af-44f8-aca6-b0b36b778b25)
- ESP32 Devkit V1
- MAX30102 SpO2 and Heartbeat Sensor
- MPU6050 IMU
- GPS Ublox Neo-6M
- OLED 128x64
- Push Button
- 10K Resistor
- Buzzer

## Operation and Features
As an usual watch, it shows the date and time in front-most display. Basicly, when the system is just getting powered, the internal ESP32 RTC is in reset. Time need to be synchronized with the NTP server at the first time.

To navigates through different functionalities, it is already comprised several menus configured with state machines, and also two push button to scroll and proceed.

Menus:
- Running Mode, shows heartrate, steps, position (lon & lat).
- Health test, heartrate and SpO2 level
- Timer
- Active/Saver Mode, activates connections to Wifi and initiates a Web Server. With saver mode, low-power operation is enabled with a deep-sleep after 30s.

Web Server:


