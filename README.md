## FreeRTOS application running on ESP32 embedded system

### Introduction
The proyect hardware consists on one ESP32 board working along with the accelerometer/gyroscope chip MPU6050. The application main objetive is to measure animals acceleration for monitoring them in the countryside.

### Goals
The main goals of the proyect are:

 * Sense animals acceleration
 * Store measurements in memory (a caché-level like system between the RAM and the filesystem)
 * Send data through WiFi when desired
 * The priors but paying attention on the power management 

### Run in one command (Linux only)
Clone the repository and go to the proyect folder, then

* Connect hardware to PC
* ```make flashandtest``` build, flash, and monitor the proyect
 
This is assuming the hardware is in the serial port /dev/ttyUSB0. If not, change it in the scripts under the scripts dir.

### Known bugs
* Sometimes, when you flash the micro, it doesn't wake up when waiting for the interrupt from the accelerometer. This can be solved reseting it. After the bug is solved, is not going to happen again until a new flash.
