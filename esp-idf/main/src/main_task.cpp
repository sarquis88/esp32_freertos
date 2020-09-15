#include "../include/main_task.h"

/* Static variables declaration */
static xQueueHandle main_reception_queue;
static xQueueHandle main_sending_queue;

/* Precompilation definitions */
#define MAIN_TASK_VERBOSITY_LEVEL      ( 1 )

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

	/* Variables initialization */
	main_reception_queue 	= xQueueCreate( 6, sizeof( uint16_t ) );
	main_sending_queue 		= xQueueCreate( 6, sizeof( uint16_t ) );

	/* Starts accelerometer task */
	start_accelerometer_task( &main_sending_queue, &main_reception_queue );

	/* Starts main task */
	prvMainTask( NULL );
	
}

void 
prvMainTask( void* pvParameters )
{
	uint8_t queue_buffer;

	for(;;) 
	{	
		/* Start receiving any message from queue */
        xQueueReceive( main_reception_queue, &queue_buffer, portMAX_DELAY );

		if( queue_buffer == CODE_ATOM_STARTTRANSFER )
		{
			uint16_t data_size, i;

			/* Send ACK message */
			queue_buffer = CODE_ACK;
			xQueueSend( main_sending_queue, &queue_buffer, portMAX_DELAY );

			/* Receive data size */
			xQueueReceive( main_reception_queue, &data_size, portMAX_DELAY );
			xQueueSend( main_sending_queue, &queue_buffer, portMAX_DELAY );
			#if MAIN_TASK_VERBOSITY_LEVEL > 0
			ESP_LOGI( MAIN_TASK_TAG, "Receiving data. Size %d", data_size );
			#endif

			/* Receive data */
			for( i = 0; i < data_size; i++ )
			{
				xQueueReceive( main_reception_queue, &queue_buffer, portMAX_DELAY );
				#if MAIN_TASK_VERBOSITY_LEVEL > 0
				ESP_LOGI( MAIN_TASK_TAG, "[%d] %d", i, queue_buffer );
				#endif

				queue_buffer = CODE_ACK;
				xQueueSend( main_sending_queue, &queue_buffer, portMAX_DELAY );	
			}
		}			
    }
}
