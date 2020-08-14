#include "../include/wifi_scanner_task.h"

void
start_wifiscanner_task()
{
    xTaskCreate( 	prvWifiScannerTask, "wifiScannerTask", 
					WIFISCANNER_TASK_STACK_SIZE, NULL, 
					WIFISCANNER_TASK_PRIORITY, NULL );
}

void
prvWifiScannerTask( void *pvParameter )
{
    string scan_info;

	nvs_flash_init();
	tcpip_adapter_init();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );
	ESP_ERROR_CHECK( esp_wifi_set_storage( WIFI_STORAGE_RAM ) );
	ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
	ESP_ERROR_CHECK( esp_wifi_start() );

    for( ; ; )
	{
		safe_cout( "WifiScanner -> Sleeping..." );
		vTaskDelay( DELAY_WIFISCANNER_MS / portTICK_PERIOD_MS);

		safe_cout( "WifiScanner -> Scanning..." );
        scan_wifi( &scan_info );

		cout << scan_info << endl;
    }

	vTaskDelete( NULL );
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

	for( i = 0; i < MAX_APS_LIST_SIZE; i++ ) 
	{
		*buffer += "WifiScanner -> SSID = ";
		*buffer += string( ( char* ) list[i].ssid );

		if( i < MAX_APS_LIST_SIZE - 1 )
			*buffer += "\n";
	}

	free(list);
}

