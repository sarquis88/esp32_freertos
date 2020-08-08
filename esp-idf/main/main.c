#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DELAY_MS						( ( uint16_t ) 500 )

#define mainCOUNT_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define configCOUNT_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE + 772 )

static void prvCountTask( void * );

void app_main(void)
{
    if ( xTaskCreate( 	prvCountTask, "countTask", 
						configCOUNT_TASK_STACK_SIZE, NULL, 
						mainCOUNT_TASK_PRIORITY, NULL )
						!= pdPASS )
	{
		vTaskStartScheduler();
	}
}

static void prvCountTask( void *pvParameters )
{
	uint8_t i;
	UBaseType_t uxHighWaterMark;

	i = 0;
	uxHighWaterMark = ( ( UBaseType_t ) ( 0 ) );

    for( ; ; )
	{
        printf("[%d] - [%d] - [%d]\n", i++, uxHighWaterMark, configCOUNT_TASK_STACK_SIZE);
        vTaskDelay( DELAY_MS / portTICK_PERIOD_MS);
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    }

	vTaskDelete( NULL );
}
