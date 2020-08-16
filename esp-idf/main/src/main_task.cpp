#include "../include/main_task.h"

/* ######################################################################## */

static SemaphoreHandle_t cout_mutex = NULL;

/* ######################################################################## */

void 
app_main( void )
{
	init_config();
	
	start_wificonnection_task();
}

/* ######################################################################## */

void
safe_cout( string msg, bool interruption )
{
	if( interruption )
		xSemaphoreTakeFromISR( cout_mutex, NULL );
	else
		xSemaphoreTake( cout_mutex, portMAX_DELAY );

	cout << msg << endl;

	if( interruption )
		xSemaphoreGiveFromISR( cout_mutex, NULL );
	else
		xSemaphoreGive( cout_mutex );
}

void
start_deep_sleep()
{
    esp_sleep_enable_ext1_wakeup( 1ULL<<GPIO_PIN_2, ESP_EXT1_WAKEUP_ANY_HIGH );
	esp_deep_sleep_start();
}

void
init_config()
{
		cout_mutex = xSemaphoreCreateBinary();
		xSemaphoreGive( cout_mutex );
}