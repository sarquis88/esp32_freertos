#include <stdio.h>
#include <iostream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

using namespace std;

#define DELAY_COUNTER_MS				( ( uint16_t ) 500 )
#define DELAY_BLINKER_MS				( ( uint16_t ) 5000 )

#define mainCOUNT_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainBLINK_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )

#define configCOUNT_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE + 1024 )
#define configBLINK_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE + 2048 )

#define GPIO_OUTPUT_IO_0   				( ( gpio_num_t ) 18 )
#define GPIO_OUTPUT_IO_1    			( ( gpio_num_t ) 19 )
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))

static void prvCountTask( void * );
static void prvBlinkTask( void * );

void configure_gpio( void );

extern "C"
{
	void app_main( void );
}

void app_main( void )
{
    xTaskCreate( 	prvCountTask, "countTask", 
					configCOUNT_TASK_STACK_SIZE, NULL, 
					mainCOUNT_TASK_PRIORITY, NULL );

	xTaskCreate( 	prvBlinkTask, "blinkTask", 
					configBLINK_TASK_STACK_SIZE, NULL, 
					mainBLINK_TASK_PRIORITY, NULL );
}

static void prvCountTask( void *pvParameters )
{
	uint8_t i;
	UBaseType_t uxHighWaterMark;

	i = 0;
	uxHighWaterMark = ( ( UBaseType_t ) ( 0 ) );

    for( ; ; )
	{
        cout << "Counter -> [" << i++ << "] - WaterMark = " << uxHighWaterMark << endl;
        vTaskDelay( DELAY_COUNTER_MS / portTICK_PERIOD_MS);
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    }

	vTaskDelete( NULL );
}

static void prvBlinkTask( void *pvParameters )
{
	UBaseType_t uxHighWaterMark;

	uxHighWaterMark = ( ( UBaseType_t ) ( 0 ) );

	configure_gpio();

    for( ; ; )
	{
        cout << "Blinker -> WaterMark = " << uxHighWaterMark << endl;

		gpio_set_level( GPIO_OUTPUT_IO_0, 1 );

        vTaskDelay( DELAY_BLINKER_MS / portTICK_PERIOD_MS);

		gpio_set_level( GPIO_OUTPUT_IO_0, 0 );

		vTaskDelay( DELAY_BLINKER_MS / portTICK_PERIOD_MS);

		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    }

	vTaskDelete( NULL );
}

void configure_gpio()
{
	gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;

    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;

    //disable pull-down mode
    io_conf.pull_down_en = ( gpio_pulldown_t ) 0;

    //disable pull-up mode
    io_conf.pull_up_en = ( gpio_pullup_t ) 0;

    //configure GPIO with the given settings
    gpio_config(&io_conf);
}