#include "../include/main_task.h"

/* ######################################################################## */

static SemaphoreHandle_t cout_mutex = NULL;

/* ######################################################################## */

void 
app_main( void )
{
	safe_cout( "Encendiendo..." );
	
	start_intreceiver_task();
	start_blinker_task();
}

/* ######################################################################## */

void
safe_cout( string msg )
{
	if( cout_mutex == NULL )
	{
		cout_mutex = xSemaphoreCreateBinary();
		xSemaphoreGive( cout_mutex );
	}

	xSemaphoreTake( cout_mutex, portMAX_DELAY );

	cout << msg << endl;

	xSemaphoreGive( cout_mutex );
}

void
start_deep_sleep()
{
    esp_sleep_enable_ext1_wakeup( 1ULL<<GPIO_PIN_2, ESP_EXT1_WAKEUP_ANY_HIGH );
	esp_deep_sleep_start();
}

/*

static void prvWifiConnectionTask	( void * );

void connect_to_ap( uint8_t* , uint8_t* );

void configure_gpio( void );
void set_pin_level( gpio_num_t, uint8_t );

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

esp_err_t
event_handler(void *ctx, system_event_t *event)
{
	if (event->event_id == SYSTEM_EVENT_STA_GOT_IP) 
	{
		printf("Our IP address is " IPSTR "\n",
		IP2STR(&event->event_info.got_ip.ip_info.ip));
		printf("We have now connected to a station and can do things...\n");
	}
	return ESP_OK;
}

*/