#include "../include/i2c_task.h"

void
writing_test()
{
    uint64_t index;

    for( index = 0; index < NVS_MAX_INDEX; index++ )
    {
        string key = NVS_STORAGE_NAME + to_string( index );

        write_to_nvs( string( NVS_STORAGE_NAME ), key, index * index * index );
    }
}

void
reading_test()
{
    uint32_t index;
    uint64_t reading;

    for( index = 0; index < NVS_MAX_INDEX; index++ )
    {
        string key = NVS_STORAGE_NAME + to_string( index );

        ESP_ERROR_CHECK( read_from_nvs( string( NVS_STORAGE_NAME ), key, &reading ) );

        ESP_LOGI( I2C_LOGGING_TAG, "%s:%llu", key.c_str(), reading );
    }

    nvs_stats_t nvs_stats;
    nvs_get_stats( NULL, &nvs_stats);
    ESP_LOGI(   I2C_LOGGING_TAG, "NVS stats: \n\t\tUsedEntries \t= %d\n\t\tFreeEntries \t= %d (126 unusable) \n\t\tAllEntries \t= %d",
                nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries );
}

void 
start_i2c_task(void)
{
    /* NVS config */
    esp_err_t err = nvs_flash_init();
    if ( err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND ) 
    {
        ESP_LOGI( I2C_LOGGING_TAG, "%s", "NVS partition was truncated and needs to be erased. Retrying..." );
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    ESP_LOGI( I2C_LOGGING_TAG, "%s", "NVS configured" );

    /* Memory test */
    writing_test();
    reading_test();

    /* I2C config */
    ESP_ERROR_CHECK( i2c_master_init() );
    ESP_LOGI( I2C_LOGGING_TAG, "%s", "Master I2C configured" );

    /* Task creation */
	xTaskCreate( 	prvI2CTask, "I2CTask", 
					I2C_TASK_STACK_SIZE, NULL, 
					I2C_TASK_PRIORITY, NULL );   
    ESP_LOGI( I2C_LOGGING_TAG, "I2C task created" );    
}

void 
prvI2CTask( void *pvParameters )
{  
    #if I2C_TASK_WATERMARK == 1
        UBaseType_t uxHighWaterMark;
    #endif

    uint8_t reception_buffer[ BUFFER_SIZE ];
    memset( reception_buffer, 0, BUFFER_SIZE );

    uint32_t buffer_index = 0;

    for(;;) 
	{           
        ESP_LOGI( I2C_LOGGING_TAG, "%s", "Receiving data..." );
        i2c_master_read_slave( I2C_NUM_0, reception_buffer, BUFFER_SIZE );

        if( (char*)reception_buffer == NULL || (char)reception_buffer[0] == '\0' ) // no more data
        {
                ESP_LOGI( I2C_LOGGING_TAG, "%s", "Empty data received" );
                ESP_LOGI( I2C_LOGGING_TAG, "%s", "Entering deep sleep mode" );
                start_deep_sleep();
        }
        else                                // data incoming
        {
                ESP_LOGI(   I2C_LOGGING_TAG, "Data number %d received: %s", 
                            buffer_index, (char*)reception_buffer );

                uint8_t data = (uint8_t) strtol( (char*)reception_buffer, NULL, 10 );

                if( data == 0 )
                {
                    ESP_LOGI( I2C_LOGGING_TAG, "%s", "Not acceptable data received" );
                    ESP_LOGI( I2C_LOGGING_TAG, "%s", "Entering deep sleep mode" );
                    start_deep_sleep();
                }
                else
                    buffer_index++;
        }        
        
        #if I2C_TASK_WATERMARK == 1
            uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
            ESP_LOGI( I2C_LOGGING_TAG, "Task WaterMark: %d", uxHighWaterMark );
        #endif
    }

	vTaskDelete( NULL );
}

esp_err_t
i2c_master_init()
{
    /* I2C config */
    i2c_config_t i2c_conf;
    i2c_conf.mode               = I2C_MODE_MASTER;
    i2c_conf.sda_io_num         = GPIO_PIN_18;
    i2c_conf.sda_pullup_en      = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_io_num         = GPIO_PIN_19;
    i2c_conf.scl_pullup_en      = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed   = I2C_MASTER_FREQ_HZ;
    i2c_param_config( I2C_NUM_0, &i2c_conf );

    return i2c_driver_install( I2C_NUM_0, i2c_conf.mode, 0, 0, 0 );
}

esp_err_t
i2c_master_read_slave( i2c_port_t i2c_num, uint8_t *reception_buffer, size_t size ) 
{
    /* Master reading */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte( cmd, (I2C_SLAVE_ADDR << 1) | 1, true );
    i2c_master_read( cmd, reception_buffer, size - 1, I2C_MASTER_ACK );
    i2c_master_read_byte( cmd, reception_buffer + size - 1, I2C_MASTER_NACK );
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin( i2c_num, cmd, 1000 / portTICK_RATE_MS );
    i2c_cmd_link_delete( cmd );

    return ret;
}

void
start_deep_sleep()
{
    esp_sleep_enable_ext1_wakeup( 1ULL<<GPIO_PIN_2, ESP_EXT1_WAKEUP_ANY_HIGH );
	esp_deep_sleep_start();
}

esp_err_t
read_from_nvs( string storage, string key, uint64_t *value )
{
    /* Open nvs */
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open( storage.c_str(), NVS_READWRITE, &my_handle );
    if ( err == ESP_OK ) 
    {
        /* Read value */
        err = nvs_get_u64( my_handle, key.c_str(), value );
        nvs_close( my_handle );
    }
    
    return err;
}

esp_err_t
write_to_nvs( string storage, string key, uint64_t value )
{
    /* Open nvs */
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open( storage.c_str(), NVS_READWRITE, &my_handle );
    if ( err == ESP_OK ) 
    {
        /* Write value */
        err = nvs_set_u64( my_handle, key.c_str(), value );
        if( err == ESP_OK )
        {
            err = nvs_commit( my_handle );
        }
        nvs_close( my_handle );
    }

    return err;
}
