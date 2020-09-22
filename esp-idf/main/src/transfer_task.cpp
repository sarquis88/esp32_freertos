// TODO: partitions_singleapp.csv

#include "../include/transfer_task.h"

/* Static variables declaration */
static xQueueHandle* transfer_task_queue;
static xQueueHandle* accelerometer_task_queue;
static bool connected, ip_available;

/* Precompilation definitions */
#define TRANSFER_TASK_VERBOSITY_LEVEL      ( 1 )

void
start_transfer_task( xQueueHandle* reception, xQueueHandle* sending)
{
	/* Asign passed queue to local one */
    transfer_task_queue = reception;
    accelerometer_task_queue = sending;

    xTaskCreate( 	prvTransferTask, "transferTask", 
                    TRANSFER_TASK_STACK_SIZE, NULL, 
                    TRANSFER_TASK_PRIORITY, NULL );
}

void 
prvTransferTask( void *pvParameters )
{
	#if TRANSFER_TASK_VERBOSITY_LEVEL > 0 
    ESP_LOGI( TRANSFER_TASK_TAG, "Task initialized" );    
    #endif

	uint32_t queue_buffer;

	connected = false;
	ip_available = false;
	
	for(;;) 
	{	
		/* Start receiving any message from queue */
        xQueueReceive( *transfer_task_queue, &queue_buffer, portMAX_DELAY );

		if( queue_buffer == CODE_STARTTRANSFER )
		{
			uint8_t *data_pointer;
			uint16_t i, data_size;
			wifi_ap_record_t ap_info;

			/* Send ACK message */
			queue_buffer = CODE_ACK;
			xQueueSend( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );

			/* Receive data size */
			xQueueReceive( *transfer_task_queue, &data_size, portMAX_DELAY );
			xQueueSend( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );

			/* Receive data pointer */
			xQueueReceive( *transfer_task_queue, &queue_buffer, portMAX_DELAY );
			data_pointer = ( uint8_t* ) queue_buffer;
			queue_buffer = CODE_ACK;
			xQueueSend( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );

			/* Wifi configuration */
			wifi_config( string( "AbortoLegalYa" ), string( "mirifea123" ) );
			#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
			ESP_LOGI( TRANSFER_TASK_TAG, "%s", "WiFi has been configured" );
			#endif

			/* Wifi connection */
			while( !connected )
			{
				if( esp_wifi_sta_get_ap_info( &ap_info ) != ESP_OK )
				{
					ESP_ERROR_CHECK( esp_wifi_connect() );
				}
				vTaskDelay( TRANSFER_TASK_DELAY / portTICK_PERIOD_MS );
			} 

			/* Waiting for WiFi connection stablished */
			while( !ip_available )
			{
				vTaskDelay( TRANSFER_TASK_DELAY_1S / portTICK_PERIOD_MS );
			}

			/* Send data */
			for( i = 0; i < data_size; i++ )
			{
				http_send( HTTP_KEY + to_string( i ), to_string( data_pointer[ i ] ) );	
			}
			
			#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
			ESP_LOGI( TRANSFER_TASK_TAG, "Data (%d) has been retranssmited from accelerometer to WiFi", data_size );
			#endif

			/* Send end message */
			queue_buffer = CODE_ENDTRANSFER;
			xQueueSend( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );
		}			
    }

	/* Exit routine. Task should not reach here */
    #if TRANSFER_TASK_VERBOSITY_LEVEL > 0
    ESP_LOGI( TRANSFER_TASK_TAG, "%s", "Task ended" );
    #endif
	vTaskDelete( NULL );
}

esp_err_t
event_handler( void *ctx, system_event_t *event )
{
	if( event->event_id == SYSTEM_EVENT_STA_GOT_IP ) 
	{
		ip_available = true;
		#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
		ESP_LOGI( TRANSFER_TASK_TAG, "IP address: " IPSTR, IP2STR( &event->event_info.got_ip.ip_info.ip ) );
		#endif
	}
	else if( event->event_id == SYSTEM_EVENT_STA_CONNECTED )
	{
		connected = true;
		#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
		ESP_LOGI( TRANSFER_TASK_TAG, "Connected to %s", (const char *)&event->event_info.connected.ssid );
		#endif
	}
	else if( event->event_id == SYSTEM_EVENT_STA_DISCONNECTED )
	{
		connected = false;
		#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
		ESP_LOGI( TRANSFER_TASK_TAG, "Disconnected" );
		#endif
	}
	
	return ESP_OK;
}

void 
wifi_config( string ssid, string passwd )
{
	wifi_init_config_t cfg;

	nvs_flash_init();
	tcpip_adapter_init();
	ESP_ERROR_CHECK( esp_event_loop_init( event_handler, NULL ) );
	cfg = WIFI_INIT_CONFIG_DEFAULT();
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
}

void
http_send( string key, string value )
{
	int8_t wlen;
	string post_data;

	post_data = "{\"" + key + "\":\"" + value + "\"}";

	#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
	esp_http_client_config_t config = {
        .url = HTTP_URL
    };
	#pragma GCC diagnostic pop

	esp_http_client_handle_t client = esp_http_client_init( &config );

	ESP_ERROR_CHECK( esp_http_client_set_method( client, HTTP_METHOD_POST ) );
    ESP_ERROR_CHECK( esp_http_client_set_header( client, "Content-Type", "application/json" ) );
    ESP_ERROR_CHECK( esp_http_client_open( client, strlen( post_data.c_str() ) ) );
    
	wlen = esp_http_client_write( client, post_data.c_str(), strlen( post_data.c_str() ) );
	#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
	if ( wlen < 0 )
		ESP_LOGE( TRANSFER_TASK_TAG, "Write failed");
	#endif

    ESP_ERROR_CHECK( esp_http_client_cleanup( client ) );
}