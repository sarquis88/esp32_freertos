#include "../include/main_task.h"

/* Precompilation definitions */
#if GLOBAL_VERBOSITY_LEVEL > 0
#define MAIN_TASK_VERBOSITY_LEVEL      ( 1 )
#endif

/* ######################################################################### */
/* ######################################################################### */

void 
app_main( void )
{	
	#if MAIN_TASK_VERBOSITY_LEVEL > 0
	ESP_LOGI( MAIN_TASK_TAG, "%s", "" );
	ESP_LOGI( MAIN_TASK_TAG, "%s", "Starting main task" );
	ESP_LOGI( MAIN_TASK_TAG, "%s", "" );
	#endif

	/* Variables declaration */
	static xQueueHandle transfer_task_queue;
	static xQueueHandle accelerometer_task_queue;

	/* Queue initialization */
	transfer_task_queue 		= xQueueCreate( QUEUE_LENGTH, QUEUE_ITEM_SIZE );
	accelerometer_task_queue 	= xQueueCreate( QUEUE_LENGTH, QUEUE_ITEM_SIZE );
	#if MAIN_TASK_VERBOSITY_LEVEL > 0
	ESP_LOGI( MAIN_TASK_TAG, "%s", "Task queues created" );
	#endif

	/* Starts tasks */
	start_accelerometer_task( &accelerometer_task_queue, &transfer_task_queue );
	start_transfer_task		( &transfer_task_queue, &accelerometer_task_queue );
	#if MAIN_TASK_VERBOSITY_LEVEL > 0
	ESP_LOGI( MAIN_TASK_TAG, "%s", "Tasks created" );
	#endif

	while( true )
	{
		vTaskDelay( 1000 / portTICK_PERIOD_MS );
	}
	
}
