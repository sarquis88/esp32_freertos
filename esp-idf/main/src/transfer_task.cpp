#include "../include/transfer_task.h"

/* Static variables declaration */
static xQueueHandle* transfer_task_queue;
static xQueueHandle* accelerometer_task_queue;
static uint16_t data_size;
static uint8_t *data_buffer;
static bool connected;

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

	uint16_t queue_buffer;

	connected = false;
	
	for(;;) 
	{	
		/* Start receiving any message from queue */
        xQueueReceive( *transfer_task_queue, &queue_buffer, portMAX_DELAY );

		if( queue_buffer == CODE_STARTTRANSFER )
		{
			uint16_t i;
			wifi_ap_record_t ap_info;

			/* Send ACK message */
			queue_buffer = CODE_ACK;
			xQueueSend( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );

			/* Receive data size */
			xQueueReceive( *transfer_task_queue, &data_size, portMAX_DELAY );
			xQueueSend( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );
			#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
			ESP_LOGI( TRANSFER_TASK_TAG, "Receiving data. Size %d", data_size );
			#endif

			/* Prepare data buffer */
			data_buffer = ( uint8_t* ) malloc( sizeof ( uint8_t ) * data_size );

			/* Receive data */
			for( i = 0; i < data_size; i++ )
			{
				/* Receive one data */
				xQueueReceive( *transfer_task_queue, &queue_buffer, portMAX_DELAY );
				#if MAIN_TASK_VERBOSITY_LEVEL > 1
				ESP_LOGI( TRANSFER_TASK_TAG, "[%d] %d", i, queue_buffer );
				#endif
				data_buffer[ i ] = ( uint8_t ) queue_buffer;

				/* Send ACK message */
				queue_buffer = CODE_ACK;
				xQueueSend( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );	
			}
			#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
			ESP_LOGI( TRANSFER_TASK_TAG, "%s", "Data has been received" );
			#endif

			/* Wifi configuration */
			wifi_config( string( "esp32" ), string( "computacion" ) );
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
			
		}			
    }

	free( data_buffer );
	vTaskDelete( NULL );
}

esp_err_t
event_handler( void *ctx, system_event_t *event )
{
	if( event->event_id == SYSTEM_EVENT_STA_GOT_IP ) 
	{
		uint16_t queue_buffer, i;

		#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
		ESP_LOGI( TRANSFER_TASK_TAG, "IP address: " IPSTR, IP2STR( &event->event_info.got_ip.ip_info.ip ) );
		#endif

		/* Socket config */
		socket_config();

		/* Send data through WiFi */
		for( i = 0; i < data_size; i++ )
		{
			send_message( data_buffer[ i ] );
		}
		#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
		ESP_LOGI( TRANSFER_TASK_TAG, "%s", "Data has been sended through WiFi" );
		#endif

		/* Send end message */
		queue_buffer = CODE_ENDTRANSFER;
		xQueueSend( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );
	}
	else if( event->event_id == SYSTEM_EVENT_STA_CONNECTED )
	{
		#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
		ESP_LOGI( TRANSFER_TASK_TAG, "Connected to %s", (const char *)&event->event_info.connected.ssid );
		#endif
		connected = true;
	}
	else if( event->event_id == SYSTEM_EVENT_STA_DISCONNECTED )
	{
		#if TRANSFER_TASK_VERBOSITY_LEVEL > 0
		ESP_LOGI( TRANSFER_TASK_TAG, "Disconnected" );
		#endif
		connected = false;
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

struct sockaddr_in dest_addr;
int sockfd;
int destination_port = 20000;
const char* destination_ip = "192.168.100.3";

void
socket_config()
{
	struct hostent *dest;

	// dest = gethostbyname( destination_ip );
	// sockfd = socket( AF_INET, SOCK_DGRAM, 0 );
	// dest_addr.sin_family = AF_INET;
	// dest_addr.sin_port = htons( destination_port );
	// dest_addr.sin_addr = *( (struct in_addr *)dest->h_addr );
	// memset( &(dest_addr.sin_zero), '\0', 8 );	

	// ESP_LOGI( TRANSFER_TASK_TAG, "Socket configured" );

	// sockfd = socket( AF_INET, SOCK_STREAM, 0);
	// memset( (char *) &serv_addr, 0, sizeof(serv_addr) );

	dest = gethostbyname( destination_ip );
	sockfd = socket( AF_INET, SOCK_STREAM, 0 );
	memset( (char *) &dest_addr, '0', sizeof(dest_addr) );
	dest_addr.sin_family = AF_INET;
	bcopy( (char *)dest->h_addr, (char *)&dest_addr.sin_addr.s_addr, (size_t )dest->h_length );
	dest_addr.sin_port = htons( (uint16_t) 20000 );

	connect( sockfd, (struct sockaddr *)&dest_addr, sizeof(dest_addr ) );
}

void
send_message( uint8_t data )
{
	ssize_t n;
	uint8_t data_buff;

	data_buff = data;

	n = send	(
				sockfd,
				&data_buff, 
				sizeof( data_buff ),
				0
				);

	if( n <= 0 )
	{
		#if TRANSFER_TASK_VERBOSITY_LEVEL > 1
		ESP_LOGE( TRANSFER_TASK_TAG, "Error sending: %d", data );
		#endif
	}
	else
	{
		#if TRANSFER_TASK_VERBOSITY_LEVEL > 1
		ESP_LOGI( TRANSFER_TASK_TAG, "Data sended: %d", data );
		#endif
	}
}