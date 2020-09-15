#include "../include/main_task.h"

// TODO: I2C in deep-sleep mode

/* ######################################################################## */

#define ACCELEROMETER_TASK_ON		1
#define TASKSLIST_TASK_ON			0

/* ######################################################################## */

void 
app_main( void )
{	
	ESP_LOGI( "", "%s", "" );
	ESP_LOGI( "", "%s", "" );
	ESP_LOGI( "", "%s", "" );

	xQueueHandle main_reception_queue;
	xQueueHandle main_sending_queue;
	char queue_buffer;

	main_reception_queue 	= xQueueCreate( 6, sizeof( char ) );
	main_sending_queue 		= xQueueCreate( 6, sizeof( char ) );

	/* Starts accelerometer senses */
	start_accelerometer_task( &main_sending_queue, &main_reception_queue );

	for(;;) 
	{
        if( xQueueReceive( main_reception_queue, &queue_buffer, portMAX_DELAY ) ) 
		{
			ESP_LOGI( MAIN_TASK_TAG, "Message received: %c", queue_buffer );

			if( queue_buffer == 'S' )
			{
				vTaskDelay( 5000 / portTICK_PERIOD_MS);
				xQueueSend( main_sending_queue, "D", portMAX_DELAY );
			}				
        }
    }
}

void
power_test()
{
	ESP_LOGI( MAIN_TASK_TAG, "%s", "I'm awake!" );
	vTaskDelay( 5000 / portTICK_PERIOD_MS);

	ESP_LOGI( MAIN_TASK_TAG, "%s", "I'm going to sleep..." );
	ESP_ERROR_CHECK( esp_sleep_enable_timer_wakeup( 5000000 ) );
	esp_deep_sleep_start();
}
