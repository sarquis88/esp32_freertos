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
	nvs_flash_init();
	tcpip_adapter_init();
	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
	
	#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
	wifi_config_t sta_config = 
	{
		.sta = 
		{
			.bssid_set = 0
		}
	};
	#pragma GCC diagnostic pop

	strcpy( (char*)sta_config.sta.ssid, "ssid" );
	strcpy( (char*)sta_config.sta.password, "password" );
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(esp_wifi_connect());

    for( ; ; )
	{
		vTaskDelay( WIFICONNECTION_DELAY_MS / portTICK_PERIOD_MS);
    }

	vTaskDelete( NULL );
}

esp_err_t
event_handler(void *ctx, system_event_t *event)
{
	char msg[ BUFFER_SIZE ];

	if( event->event_id == SYSTEM_EVENT_STA_CONNECTED )
	{
		sprintf( msg, "WifiConnection -> Connected" );
		safe_cout( string( msg ), true );
	}
	else if( event->event_id == SYSTEM_EVENT_STA_GOT_IP ) 
	{
		sprintf( msg, "WifiConnection -> IP address: " IPSTR "\n", IP2STR( &event->event_info.got_ip.ip_info.ip ) );
		safe_cout( string( msg ), true );
	}
	else if( event->event_id == SYSTEM_EVENT_STA_DISCONNECTED )
	{
		sprintf( msg, "WifiConnection -> Disconnected" );
		safe_cout( string( msg ), true );
	}
	return ESP_OK;
}