#include <stdio.h>
#include <iostream>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_log.h>

#include <nvs_flash.h>
#include <tcpip_adapter.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include "sdkconfig.h"


using namespace std;

#define GPIO_PIN_2                      ( ( gpio_num_t ) 2 		)
#define GPIO_PIN_18   					( ( gpio_num_t ) 18 	)
#define GPIO_PIN_19   					( ( gpio_num_t ) 19 	)

#define BUFFER_SIZE						( 64 					)

#define MAIN_LOGGING_TAG              	( ( const char * ) "Main" )

extern "C"
{
	void app_main	( void );
}

extern void start_intreceiver_task         	( void      );
extern void start_wifiscanner_task         	( void      );
extern void start_wificonnection_task      	( void      );
extern void start_blinker_task         		( void      );
extern void start_taskslist_task           	( void		);
extern void start_i2c_task		           	( void		);
extern void safe_cout						( string 	);
extern void start_deep_sleep				( void 		);

void start_deep_sleep   ( void 			);
void init_config		( void			);