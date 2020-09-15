#include "freertos/FreeRTOS.h"
#include "driver/rtc_io.h"

#include <esp_sleep.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>

#include <nvs_flash.h>
#include <tcpip_adapter.h>

#include <iostream>
#include <cstring>
#include <netdb.h>

#include "../lib/MPU6050/MPU6050.h"

using namespace std;

#define GPIO_PIN_2                      ( ( gpio_num_t ) 2 			)
#define GPIO_PIN_18   					( ( gpio_num_t ) 18 		)
#define GPIO_PIN_19   					( ( gpio_num_t ) 19 		)

#define MAIN_TASK_TAG          			( ( const char* ) "Main"   	)

#define CODE_ATOM_STARTTRANSFER			( ( uint8_t ) 0			)
#define CODE_ACK						( ( uint8_t ) 2			)

extern "C"
{
	void app_main	( void );
}

extern void start_accelerometer_task    ( xQueueHandle*, xQueueHandle*	);
extern void start_transfer_task    		( xQueueHandle*, xQueueHandle*	);

void prvMainTask						( void*							);

