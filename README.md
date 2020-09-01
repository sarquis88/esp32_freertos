## FreeRTOS application running on ESP32 embedded system

### Introduction
    The proyect hardware consists on one ESP32 board working along with the accelerometer/gyroscope chip MPU6050. The application main objetive is to measure animals acceleration for monitoring them in the countryside.

### Goals
    The main goals of the proyect are:
    * Sense animals acceleration
    * Store measurements in NVS (Non-Volatile-Storage)
    * Send data through WiFi when storage is full
    * The priors but paying attention to the power management 

### Flashing the proyect to ESP32
    * Connect hardware to PC
    * ```make flash``` build and flash the proyect
    * ```make monitor``` monitor the running
    esp32-app
