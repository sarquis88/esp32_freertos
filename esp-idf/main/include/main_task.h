#include <iostream>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/rtc_io.h"

#include <esp_sleep.h>
#include <esp_log.h>

#include "../lib/MPU6050/MPU6050.h"
#include "../lib/half_float/umHalf.h"

using namespace std;

#define GPIO_PIN_2                      ( ( gpio_num_t ) 2 		)
#define GPIO_PIN_16   					( ( gpio_num_t ) 16 	)
#define GPIO_PIN_17   					( ( gpio_num_t ) 17 	)
#define GPIO_PIN_18   					( ( gpio_num_t ) 18 	)
#define GPIO_PIN_19   					( ( gpio_num_t ) 19 	)
#define GPIO_PIN_21   					( ( gpio_num_t ) 21 	)
#define GPIO_PIN_25   					( ( gpio_num_t ) 25 	)
#define GPIO_PIN_33   					( ( gpio_num_t ) 33 	)

#define BUFFER_SIZE						( 64 					)

#define MAIN_LOGGING_TAG              	( ( const char * ) "Main" )

extern "C"
{
	void app_main	( void );
}

extern void start_taskslist_task           	( void		);
extern void start_accelerometer_task       	( void		);
