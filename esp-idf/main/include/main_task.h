#include "freertos/FreeRTOS.h"
#include "driver/rtc_io.h"

#include <esp_sleep.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_http_client.h>

#include <nvs_flash.h>
#include "esp_spiffs.h"
#include <tcpip_adapter.h>

#include <iostream>
#include <fstream>
#include <cstring>
#include <netdb.h>

#include "../lib/MPU6050/MPU6050.h"

using namespace std;

#define GPIO_PIN_2                      ( ( gpio_num_t ) 2 						)
#define GPIO_PIN_18   					( ( gpio_num_t ) 18 					)
#define GPIO_PIN_19   					( ( gpio_num_t ) 19 					)

#define GLOBAL_VERBOSITY_LEVEL			( 1										)

#define MAIN_TASK_TAG          			( ( const char* ) "Main"   				)

#define CODE_STARTTRANSFER				( ( uint8_t ) 0							)
#define CODE_ENDTRANSFER				( ( uint8_t ) 1							)
#define CODE_ACK						( ( uint8_t ) 2							)

#define QUEUE_LENGTH					( ( UBaseType_t ) 5						)
#define QUEUE_ITEM_SIZE					( ( UBaseType_t ) sizeof( uint32_t )	)

#define SPIFFS_BASE_PATH            	( ( const char * ) "/spiffs"            )
#define SPIFFS_FILE_NAME            	( ( const char * ) "/spiffs/accel.txt"	)

extern "C"
{
	/*
		Main task function, the first one who starts
		This function is in charge of starting the other tasks and preparing
		its queues for passing messages
	*/
	void app_main	( void );
}

extern void start_accelerometer_task    ( xQueueHandle*, xQueueHandle*	);
extern void start_transfer_task    		( xQueueHandle*, xQueueHandle*	);