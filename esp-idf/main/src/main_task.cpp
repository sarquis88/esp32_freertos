#include "../include/main_task.h"

/* ######################################################################## */

/*
#define BLINKER_TASK_ON				0
#define INTRECEIVER_TASK_ON			0
#define WIFICONNECTION_TASK_ON		0
#define WIFISCANNER_TASK_ON			0
*/

/* ######################################################################## */

void 
app_main( void )
{	
	init_config();

	start_wificonnection_task();
}

/* ######################################################################## */

void
start_deep_sleep()
{
    esp_sleep_enable_ext1_wakeup( 1ULL<<GPIO_PIN_2, ESP_EXT1_WAKEUP_ANY_HIGH );
	esp_deep_sleep_start();
}

void
init_config()
{
	ESP_LOGI( MAIN_LOGGING_TAG, "Initial configuration...");
}