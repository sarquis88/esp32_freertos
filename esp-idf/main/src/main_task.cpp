#include "../include/main_task.h"

/* ######################################################################## */

void 
app_main( void )
{	
	start_wificonnection_task();
}

/* ######################################################################## */

void
start_deep_sleep()
{
    esp_sleep_enable_ext1_wakeup( 1ULL<<GPIO_PIN_2, ESP_EXT1_WAKEUP_ANY_HIGH );
	esp_deep_sleep_start();
}