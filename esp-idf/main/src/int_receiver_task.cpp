#include "../include/int_receiver_task.h"

static xQueueHandle gpio_int_queue = NULL;

void
start_intreceiver_task()
{
	/* Interruption queue */
	gpio_int_queue = xQueueCreate( 10, sizeof(uint32_t) );

	/* Input config */
	gpio_config_t io_conf;
	io_conf.pull_down_en 	= ( gpio_pulldown_t ) 0;
	io_conf.intr_type 		= GPIO_INTR_NEGEDGE;
    io_conf.mode 			= GPIO_MODE_INPUT;
    io_conf.pin_bit_mask 	= 1ULL<<GPIO_PIN_19;
    io_conf.pull_up_en 		= ( gpio_pullup_t ) 1;
    ESP_ERROR_CHECK( gpio_config( &io_conf ) );
    ESP_ERROR_CHECK( gpio_install_isr_service( 0 ) );
    ESP_ERROR_CHECK( gpio_isr_handler_add( GPIO_PIN_19, gpio_int_handler, (void*) 88 ) );

	/* Task creation */
	xTaskCreate( 	prvIntReceiverTask, "intReceiverTask", 
					configINTRECEIVER_TASK_STACK_SIZE, NULL, 
					mainINTRECEIVER_TASK_PRIORITY, NULL );
}

void
IRAM_ATTR gpio_int_handler( void* arg )
{
	uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR( gpio_int_queue, &gpio_num, NULL);
}

void 
prvIntReceiverTask( void *pvParameters )
{
	uint32_t buffer;
	uint8_t c;
	string message;

	c = 0;

    for(;;) 
	{
        if( xQueueReceive( gpio_int_queue, &buffer, portMAX_DELAY ) ) 
		{
			safe_cout( "Apagando..." );
			start_deep_sleep();

			// message = "IntReceiver -> [] Interruption received";
            // safe_cout( message.insert( 16, to_string( c++ ) ) );
        }
    }

	vTaskDelete( NULL );
}