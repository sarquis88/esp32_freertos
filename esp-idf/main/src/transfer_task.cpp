#include "../include/transfer_task.hpp"

/* Static variables declaration */
static xQueueHandle* transfer_task_queue;
static xQueueHandle* accelerometer_task_queue;
static bool connected, ip_available;

/* Precompilation definitions */
#if GLOBAL_VERBOSITY_LEVEL > 0
#define TRANSFER_TASK_VERBOSITY_LEVEL      ( 1 )
#endif

void
start_transfer_task( xQueueHandle* reception, xQueueHandle* sending)
{
	/* Asign passed queues to local ones */
    transfer_task_queue = reception;
    accelerometer_task_queue = sending;

	/* Task creation */
    xTaskCreate( 	prvTransferTask, "transferTask", 
                    TRANSFER_TASK_STACK_SIZE, NULL, 
                    TRANSFER_TASK_PRIORITY, NULL );
}

void 
prvTransferTask( void *pvParameters )
{
	/* Log tag initialization */
	#if TRANSFER_TASK_VERBOSITY_LEVEL > 0 
    ESP_LOGI( TRANSFER_TASK_TAG, "Task initialized" );    
    #endif

	/* Variable declarations */
	uint32_t queue_buffer;

	/* Variable initialization */
	connected = false;
	ip_available = false;
	
	for(;;) 
	{	
		/* Start receiving any message from queue */
        xQueueReceive( *transfer_task_queue, &queue_buffer, portMAX_DELAY );
		
		if( queue_buffer == CODE_STARTTRANSFER )
		{
			/* Variable declarations */
			uint8_t data_chunk[ HTTP_CHUNK_SIZE ];
			uint32_t chunk_cuantity, i, data_size;
			FILE* file;
			esp_err_t err;

			/* Send ACK message */
			queue_buffer = CODE_ACK;
			xQueueSend( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );

			/* Receive data size */
			xQueueReceive( *transfer_task_queue, &queue_buffer, portMAX_DELAY );
			data_size = queue_buffer;

			/* Send ACK message */
			queue_buffer = CODE_ACK;
			xQueueSend( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );

			/* Wifi configuration */
			wifi_config( string( WIFI_SSID ), string( WIFI_PASSWD ) );
			#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
			ESP_LOGI( TRANSFER_TASK_TAG, "%s", "WiFi has been configured" );
			#endif

			/* Wifi connection */
			connection:
			ESP_ERROR_CHECK( esp_wifi_connect() );
			vTaskDelay( TRANSFER_TASK_DELAY / portTICK_PERIOD_MS );
			if( !connected )
			{
				ESP_ERROR_CHECK( esp_wifi_stop() );
				vTaskDelay( TRANSFER_TASK_DELAY / portTICK_PERIOD_MS );
				ESP_ERROR_CHECK( esp_wifi_start() );
				goto connection;
			}

			/* Mount filesystem partition */
			esp_vfs_spiffs_conf_t conf = 
			{
				.base_path = FILESYSTEM_BASE_PATH,
				.partition_label = NULL,
				.max_files = 5,
				.format_if_mount_failed = true
			};
			err = esp_vfs_spiffs_register( &conf );
			if( err != ESP_OK && err != ESP_ERR_INVALID_STATE )
			{
				#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
				ESP_LOGE( TRANSFER_TASK_TAG, "Error mounting the filesystem" );
				#endif
				return;
			}

			/* Open data file */
			file = fopen( FILESYSTEM_FILE_NAME, "r" );
			if (file == NULL) 
			{
				#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
				ESP_LOGE( TRANSFER_TASK_TAG, "Error opening file for reading" );
				#endif
				return;
			}

			/* Waiting for WiFi connection stablished */
			while( !ip_available )
			{
				vTaskDelay( TRANSFER_TASK_DELAY / portTICK_PERIOD_MS );
			}

			/* Send data as blob (chunk by chunk) */
			chunk_cuantity = data_size / HTTP_CHUNK_SIZE; 
			for( i = 0; i < chunk_cuantity; i++ )
			{	
				if( fread( (void*) data_chunk, sizeof( uint8_t ), HTTP_CHUNK_SIZE, file ) != HTTP_CHUNK_SIZE )
				{
					#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
					ESP_LOGE( TRANSFER_TASK_TAG, "Error reading the data file" );
					#endif
				}
				http_send_plain( data_chunk, HTTP_CHUNK_SIZE );
			}

			/* Check for data not sended */
			if( data_size > chunk_cuantity * HTTP_CHUNK_SIZE )
			{
				uint32_t remainder = data_size - chunk_cuantity * HTTP_CHUNK_SIZE;
				if( fread( (void*) data_chunk, sizeof( uint8_t ), remainder, file ) != remainder )
				{
					#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
					ESP_LOGE( TRANSFER_TASK_TAG, "Error reading the data file" );
					#endif
				}
				http_send_plain( data_chunk, remainder );
			}

			/* Close file and log */
			fclose( file );
			#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
			ESP_LOGI( TRANSFER_TASK_TAG, "Data (%d) has been retranssmited from accelerometer through WiFi", data_size );
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

/* ######################################################################### */
/* ######################################################################### */

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

/* ######################################################################### */
/* ######################################################################### */

void
http_send_plain( uint8_t* data, uint16_t len )
{
	int8_t wlen;

	#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
	esp_http_client_config_t config = {
        .url = HTTP_URL
    };
	#pragma GCC diagnostic pop

	esp_http_client_handle_t client = esp_http_client_init( &config );
	ESP_ERROR_CHECK( esp_http_client_set_method( client, HTTP_METHOD_POST ) );
    ESP_ERROR_CHECK( esp_http_client_set_header( client, "Content-Type", "text/plain" ) );
		
	/* If server is not found, keep trying */
	while( esp_http_client_open( client, len ) != ESP_OK )
	{
		vTaskDelay( TRANSFER_TASK_DELAY / portTICK_PERIOD_MS );
	}
    
	wlen = esp_http_client_write( client, ( char* ) data, len );
	#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
	if ( wlen < 0 )
		ESP_LOGE( TRANSFER_TASK_TAG, "Write failed" );
	#endif

    ESP_ERROR_CHECK( esp_http_client_cleanup( client ) );
}
