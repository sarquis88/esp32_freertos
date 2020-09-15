#include "freertos/FreeRTOS.h"
#include "driver/rtc_io.h"

#include <esp_sleep.h>
#include <esp_log.h>

#include "../lib/MPU6050/MPU6050.h"

using namespace std;

#define GPIO_PIN_2                      ( ( gpio_num_t ) 2 		)
#define GPIO_PIN_18   					( ( gpio_num_t ) 18 	)
#define GPIO_PIN_19   					( ( gpio_num_t ) 19 	)

#define MAIN_TASK_TAG          			( ( const char* ) "Main"   )

extern "C"
{
	void app_main	( void );
}

extern void start_accelerometer_task    ( xQueueHandle*, xQueueHandle*	);

void power_test							( void 							);
