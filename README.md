# Project Specifications 

This Project Implements an Activity Monitor application using an STM32F4 microcontroller. The goal of this project is to track a user's current activity using a BNO055 IMU sensor. The user's current activity will be displayed on two I2C LCD screens. 
- LCD1: Shows the user's current activity status such as running, walking, or being idle.
- LCD2: Displays other general information like current speed, compass direction, and temperature.

# Software 

- STM32CubeIDE

# Hardware 

- STM32F407VG microcontroller: https://www.st.com/en/evaluation-tools/stm32f4discovery.html
- IMU board BNO055: https://www.amazon.com/Adafruit-Absolute-Orientation-Fusion-Breakout/dp/B017PEIGIG
- LCD Board x2:
https://www.amazon.com/GeeekPi-Interface-Backlight-Raspberry-Electrical/dp/B0BCWJWKG2/ref=sr_1_13?crid=24R54R126XX9A&dib=eyJ2IjoiMSJ9.5CzWowNn9_mJcwMz5cfdJ6vRzkQQSzbfUZeNpPlCO0zWsBF1txbU84q34p7chboGKKC54SXfjtFeaYd1FMVZ2FXUvPRN6y_FEEdalMSmHLGNV6Vulfr5voj5GAeB3TIqPiPymswL7u1Ut09ZKO68sydUW-42zk2Oh5gIVf9_ff3Heyen23jIsmUq0xjHHQPH324zWLtaO67nM9jXm4js_ukkitclD3PkQVrg4Aajw.6LxriJ1MbDQtaHk8qjdWq73FHOhN4U_HEomh5EHoev8&dib_tag=se&keywords=lcd+board+i2sc&qid=1718427379&sprefix=lcd+board+i2sc%2Caps%2C134&sr=8-13
- Breadboard
- Jumper wires
- Resistors: 100 ohms
- Battery Pack


# Connections

- PB6: 12C1 SCL
- PB7: 12C1 SDA
- LCD1 (0x27): Connected via I2C1
- LCD2 (0x20): Connected via I2C1
- BNO055 IMU Sensor (0x28) : Connected via I2C1

# Circuit Diagram

![Circuit Diagram](https://github.com/huthifa22/STM32F407VG-Activity-Monitor/assets/105901978/05a5a8cb-4cca-493c-a396-0c8077aa24bf)

![SC1](https://github.com/huthifa22/STM32F407VG-Activity-Monitor/assets/105901978/afffdd73-1f6f-4637-aa46-a9d8acf6c655)



