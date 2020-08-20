#include "../include/wifi_connection_task.h"

struct sockaddr_in dest_addr;

int sockfd;

int destination_port = 20000;

const char* destination_ip = "192.168.100.3";

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

	wifi_config( string( "AbortoLegalYa" ), string( "mirifea123" ) );
	socket_config();

    for( ; ; )
	{
		
		if( esp_wifi_sta_get_ap_info( &ap_info ) != ESP_OK )
		{
			ESP_LOGI( WIFICONNECTION_LOGGING_TAG, "Connecting..." );
			ESP_ERROR_CHECK( esp_wifi_connect() );
		}

		vTaskDelay( WIFICONNECTION_LONG_DELAY_MS / portTICK_PERIOD_MS);
		send_message( string( "[HOLA MUNDO]" ) );
    }

	vTaskDelete( NULL );
}

esp_err_t
event_handler(void *ctx, system_event_t *event)
{
	if( event->event_id == SYSTEM_EVENT_STA_GOT_IP ) 
	{
		ESP_LOGI( WIFICONNECTION_LOGGING_TAG, "IP address: " IPSTR, IP2STR( &event->event_info.got_ip.ip_info.ip ) );
	}
	else if( event->event_id == SYSTEM_EVENT_STA_CONNECTED )
	{
		ESP_LOGI( WIFICONNECTION_LOGGING_TAG, "Connected to %s", (const char *)&event->event_info.connected.ssid );
	}
	else if( event->event_id == SYSTEM_EVENT_STA_DISCONNECTED )
	{
		ESP_LOGI( WIFICONNECTION_LOGGING_TAG, "Disconnected" );
	}
	
	return ESP_OK;
}

void 
wifi_config( string ssid, string passwd )
{
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

	strcpy( (char*)sta_config.sta.ssid, ssid.c_str() );
	strcpy( (char*)sta_config.sta.password, passwd.c_str() );
	ESP_ERROR_CHECK( esp_wifi_set_config( WIFI_IF_STA, &sta_config ) );
	ESP_ERROR_CHECK( esp_wifi_start() );

	ESP_LOGI( WIFICONNECTION_LOGGING_TAG, "WiFi configured" );
}

void
socket_config()
{
	struct hostent *dest;

	dest = gethostbyname( destination_ip );
	sockfd = socket( AF_INET, SOCK_DGRAM, 0 );
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons( destination_port );
	dest_addr.sin_addr = *( (struct in_addr *)dest->h_addr );
	memset( &(dest_addr.sin_zero), '\0', 8 );	

	ESP_LOGI( WIFICONNECTION_LOGGING_TAG, "Socket configured" );
}

void
send_message( string message )
{
	int n;
	n = sendto	(
				sockfd,
				(void *)message.c_str(), 
				message.size(),
				0,
				(struct sockaddr *)&dest_addr,
				sizeof( dest_addr ) 
				);

	if( n <= 0 )
		ESP_LOGE( WIFICONNECTION_LOGGING_TAG, "Error sending message to %s", destination_ip );
	else
		ESP_LOGI( WIFICONNECTION_LOGGING_TAG, "Message sended to %s", destination_ip );
}