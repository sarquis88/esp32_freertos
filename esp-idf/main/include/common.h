#include <stdio.h>
#include <iostream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <nvs_flash.h>
#include <tcpip_adapter.h>

using namespace std;

void safe_cout( string );
