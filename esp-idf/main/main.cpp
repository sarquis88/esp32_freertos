#include <stdio.h>
#include <iostream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <nvs_flash.h>
#include <tcpip_adapter.h>

using namespace std;

#define DELAY_WIFISCANNER_MS					( ( uint16_t ) 5000 )
#define mainWIFISCANNER_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define configWIFISCANNER_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE + 2024 )
#define MAX_APS_LIST_SIZE						3

#define GPIO_PIN_18   					( ( gpio_num_t ) 18 )
#define GPIO_PIN_19   					( ( gpio_num_t ) 19 )
#define GPIO_OUTPUT_PIN_SEL  			( 1ULL<<GPIO_PIN_18 | 1ULL<<GPIO_PIN_19 )

#define WATERMARK_ANALISIS				0

/* ######################################################################## */
/* 						FUNCTIONS DECLARATIONS								*/
/* ######################################################################## */

static void prvWifiScannerTask( void * );
static void prvWifiConnectionTask( void * );

void scan_wifi( string* );
void connect_to_ap( uint8_t* , uint8_t* );

void configure_led( void );
void set_led_level( gpio_num_t, uint8_t );

extern "C"
{
	void app_main( void );
}

/* ######################################################################## */
/* 							      APP_MAIN									*/
/* ######################################################################## */

esp_err_t event_handler(void *ctx, system_event_t *event)
{
	if (event->event_id == SYSTEM_EVENT_STA_GOT_IP) 
	{
		printf("Our IP address is " IPSTR "\n",
		IP2STR(&event->event_info.got_ip.ip_info.ip));
		printf("We have now connected to a station and can do things...\n");
	}
	return ESP_OK;
}

void 
app_main( void )
{
	TaskHandle_t wifiScannerTaskHandle, wifiConnectionTaskHandle;

    xTaskCreate( 	prvWifiScannerTask, "wifiScannerTask", 
					configWIFISCANNER_TASK_STACK_SIZE, NULL, 
					mainWIFISCANNER_TASK_PRIORITY, &wifiScannerTaskHandle );
	// xTaskCreate( 	prvWifiConnectionTask, "wifiConnectionTask", 
	// 				configWIFISCANNER_TASK_STACK_SIZE, NULL, 
	// 				mainWIFISCANNER_TASK_PRIORITY, &wifiConnectionTaskHandle );
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

	nvs_flash_init();
	tcpip_adapter_init();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );
	ESP_ERROR_CHECK( esp_wifi_set_storage( WIFI_STORAGE_RAM ) );
	ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
	ESP_ERROR_CHECK( esp_wifi_start() );

    for( ; ; )
	{
		cout << "WifiScanner -> Sleeping..." << endl;
		vTaskDelay( DELAY_WIFISCANNER_MS / portTICK_PERIOD_MS);

		cout << "WifiScanner -> Scanning..." << endl;
        scan_wifi( &scan_info );

		cout << scan_info << endl;

		#if ( WATERMARK_ANALISIS == 1 )	
			cout << "WifiScanner -> WaterMark = " << uxHighWaterMark << endl << endl;
			uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		#endif
    }

	vTaskDelete( NULL );
}

static void 
prvWifiConnectionTask( void *pvParameters )
{
	#if ( WATERMARK_ANALISIS == 1 )	
		UBaseType_t uxHighWaterMark;
		uxHighWaterMark = ( ( UBaseType_t ) 0 );
	#endif
	
	connect_to_ap( (uint8_t*) "TCLD55\0", (uint8_t*) "jujujuju\0" );

    for( ; ; )
	{
		vTaskDelay( DELAY_WIFISCANNER_MS / portTICK_PERIOD_MS);
		#if ( WATERMARK_ANALISIS == 1 )	
			cout << "WifiConnection -> WaterMark = " << uxHighWaterMark << endl << endl;
			uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		#endif
    }

	vTaskDelete( NULL );
}

/* ######################################################################## */
/* 							      FUNCTIONS									*/
/* ######################################################################## */

/* WiFi */

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

	for( i = 0; i < MAX_APS_LIST_SIZE; i++ ) 
	{
		*buffer += "WifiScanner -> SSID = ";
		*buffer += string( ( char* ) list[i].ssid );

		if( i < MAX_APS_LIST_SIZE - 1 )
			*buffer += "\n";
	}

	free(list);
}

void
connect_to_ap( uint8_t* ssid_arg, uint8_t* password_arg )
{
	nvs_flash_init();
	tcpip_adapter_init();
	ESP_ERROR_CHECK( esp_event_loop_init( event_handler, NULL ) );
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );
	ESP_ERROR_CHECK( esp_wifi_set_storage( WIFI_STORAGE_RAM ) );
	ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );

	#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
	#pragma GCC diagnostic ignored "-Wconversion-null"
	wifi_config_t sta_config = 
	{
		.sta = 
		{
			ssid_arg[0],
			password_arg[0],
			NULL,
			false,
			NULL,
			0,
			0,
			NULL,
			NULL,
			NULL
		}
	};
	#pragma GCC diagnostic pop
	#pragma GCC diagnostic pop	
	
	ESP_ERROR_CHECK( esp_wifi_set_config( WIFI_IF_STA, &sta_config ) );
	ESP_ERROR_CHECK( esp_wifi_start() );
	ESP_ERROR_CHECK( esp_wifi_connect() );
}

/* GPIO */

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
set_led_level( gpio_num_t pin, uint8_t state )
{
	gpio_set_level( pin, state );
}