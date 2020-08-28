#include "../include/blinker_task.h"

void
start_blinker_task()
{
	/* Output config */
	gpio_config_t io_conf;
	io_conf.pull_down_en 	= ( gpio_pulldown_t ) 0;
	io_conf.intr_type 		= GPIO_INTR_DISABLE;
    io_conf.mode 			= GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask 	= 1ULL<<GPIO_PIN_18;
    io_conf.pull_up_en 		= ( gpio_pullup_t ) 0;
    ESP_ERROR_CHECK( gpio_config( &io_conf ) );

	/* Task creation */
	xTaskCreate( 	prvBlinkerTask, "blinkerTask", 
					BLINKER_TASK_STACK_SIZE, NULL, 
					BLINKER_TASK_PRIORITY, NULL );
}

void 
prvBlinkerTask( void *pvParameters )
{
	uint8_t c;

	c = 0;

    for(;;) 
	{   
        gpio_set_level( GPIO_PIN_18, c % 2 );
        vTaskDelay( BLINKER_DELAY_MS / portTICK_PERIOD_MS);
        c++;
    }

	vTaskDelete( NULL );
}