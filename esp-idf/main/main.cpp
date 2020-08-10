#include <stdio.h>
#include <iostream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

using namespace std;

#define DELAY_WifiScanner_MS					( ( uint16_t ) 5000 )
#define mainWIFISCANNER_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define configWIFISCANNER_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE + 2024 )

#define GPIO_PIN_18   					( ( gpio_num_t ) 18 )
#define GPIO_OUTPUT_PIN_SEL  			( 1ULL<<GPIO_PIN_18 )

#define WATERMARK_ANALISIS				1

static void prvWifiScannerTask( void * );

void start_wifi( void );
void scan_wifi( string* );

void configure_led( void );
void set_led_level( uint8_t );

extern "C"
{
	void app_main( void );
}

/* ######################################################################## */
/* 							      APP_MAIN									*/
/* ######################################################################## */

void 
app_main( void )
{
	TaskHandle_t wifiScannerTaskHandle;

    xTaskCreate( 	prvWifiScannerTask, "wifiScannerTask", 
					configWIFISCANNER_TASK_STACK_SIZE, NULL, 
					mainWIFISCANNER_TASK_PRIORITY, &wifiScannerTaskHandle );
}

/* ######################################################################## */
/* 							      TASKS										*/
/* ######################################################################## */

static void 
prvWifiScannerTask( void *pvParameters )
{
	string scan_info;

	#if ( WATERMARK_ANALISIS == 1 )	
		UBaseType_t uxHighWaterMark;
		uxHighWaterMark = ( ( UBaseType_t ) 0 );
	#endif

	configure_led();
	start_wifi();

    for( ; ; )
	{
		cout << "WifiScanner -> Sleeping..." << endl;
		vTaskDelay( DELAY_WifiScanner_MS / portTICK_PERIOD_MS);

		cout << "WifiScanner -> Scanning..." << endl;
		set_led_level( ( uint8_t ) 1 );
        scan_wifi( &scan_info );
		set_led_level( ( uint8_t ) 0 );

		cout << scan_info << endl;

		#if ( WATERMARK_ANALISIS == 1 )	
			cout << "WifiScanner -> WaterMark = " << uxHighWaterMark << endl << endl;
			uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		#endif
    }

	vTaskDelete( NULL );
}

/* ######################################################################## */
/* 							      FUNCTIONS									*/
/* ######################################################################## */

void 
start_wifi()
{
	nvs_flash_init();
	tcpip_adapter_init();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );
	ESP_ERROR_CHECK( esp_wifi_set_storage( WIFI_STORAGE_RAM ) );
	ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
	ESP_ERROR_CHECK( esp_wifi_start() );
}

void
scan_wifi( string *buffer )
{
	uint16_t scaned_aps, i;
	*buffer = "";

	#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
	wifi_scan_config_t scanConf = 
	{
	.ssid = NULL,
	.bssid = NULL,
	.channel = 0,
	.show_hidden = 1,
	.scan_type = WIFI_SCAN_TYPE_PASSIVE
	};
	#pragma GCC diagnostic pop
	ESP_ERROR_CHECK( esp_wifi_scan_start( &scanConf, true ) );

	esp_wifi_scan_get_ap_num( &scaned_aps );

	*buffer += "WifiScanner -> Number of access points found: ";
	*buffer += to_string( scaned_aps );
	*buffer += "\n";

	if ( scaned_aps == 0 ) 
	{
		return;
	}

	wifi_ap_record_t *list = ( wifi_ap_record_t * ) malloc ( sizeof( wifi_ap_record_t ) * scaned_aps);
	ESP_ERROR_CHECK( esp_wifi_scan_get_ap_records( &scaned_aps, list ) );

	for( i = 0; i < scaned_aps; i++ ) 
	{
		*buffer += "WifiScanner -> SSID = ";
		*buffer += string( ( char* ) list[i].ssid );

		if( i < scaned_aps - 1 )
			*buffer += "\n";
	}

	free(list);
}

void
configure_led()
{
	gpio_config_t io_conf;
    io_conf.intr_type 		= GPIO_INTR_DISABLE;
    io_conf.mode 			= GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask 	= GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en 	= ( gpio_pulldown_t ) 0;
    io_conf.pull_up_en 		= ( gpio_pullup_t ) 0;
    gpio_config( &io_conf );
}

void
set_led_level( uint8_t state )
{
	gpio_set_level( GPIO_PIN_18, state );
}