#include "../include/wifi_connection_task.h"

void
start_wificonnection_task()
{
    xTaskCreate( 	prvWifiConnectionTask, "wifiConnectionTask", 
                    WIFICONNECTION_TASK_STACK_SIZE, NULL, 
                    WIFICONNECTION_TASK_PRIORITY, NULL );
}


void 
prvWifiConnectionTask( void *pvParameters )
{
	wifi_ap_record_t ap_info;

	nvs_flash_init();
	tcpip_adapter_init();
	ESP_ERROR_CHECK( esp_event_loop_init( event_handler, NULL ) );
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );
	ESP_ERROR_CHECK( esp_wifi_set_storage( WIFI_STORAGE_RAM ) );
	ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
	
	#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
	wifi_config_t sta_config = 
	{
		.sta = 
		{
			.bssid_set = 0
		}
	};
	#pragma GCC diagnostic pop

	strcpy( (char*)sta_config.sta.ssid, "esp32" );
	strcpy( (char*)sta_config.sta.password, "computacion" );
	ESP_ERROR_CHECK( esp_wifi_set_config( WIFI_IF_STA, &sta_config ) );
	ESP_ERROR_CHECK( esp_wifi_start() );

    for( ; ; )
	{
		
		if( esp_wifi_sta_get_ap_info( &ap_info ) != ESP_OK )
		{
			ESP_LOGI( WIFICONNECTION_LOGGING_TAG, "Connecting..." );
			ESP_ERROR_CHECK( esp_wifi_connect() );
		}

		vTaskDelay( WIFICONNECTION_LONG_DELAY_MS / portTICK_PERIOD_MS);
    }

	vTaskDelete( NULL );
}

esp_err_t
event_handler(void *ctx, system_event_t *event)
{
	if( event->event_id == SYSTEM_EVENT_STA_GOT_IP ) 
	{
		ESP_LOGI( WIFICONNECTION_LOGGING_TAG, "IP address: " IPSTR "\n", IP2STR( &event->event_info.got_ip.ip_info.ip ) );
	}
	else if( event->event_id == SYSTEM_EVENT_STA_CONNECTED )
	{
		ESP_LOGI( WIFICONNECTION_LOGGING_TAG, "Connected to %s", (const char *)&event->event_info.connected.ssid );
	}
	
	return ESP_OK;
}