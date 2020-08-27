#include "../include/i2c_task.h"

void
writing_test()
{
    uint64_t index;

    for( index = 0; index < NVS_MAX_INDEX; index++ )
    {
        string key = NVS_STORAGE_NAME + to_string( index );

        nvs_write( string( NVS_STORAGE_NAME ), key, index * index * index );
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

        ESP_ERROR_CHECK( nvs_read( string( NVS_STORAGE_NAME ), key, &reading ) );

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
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    ESP_LOGI( I2C_LOGGING_TAG, "%s", "NVS configured" );

    /* I2C config */
    ESP_ERROR_CHECK( i2c_init() );
    ESP_LOGI( I2C_LOGGING_TAG, "%s", "Bus configured" );

    /* MPU config */
    ESP_ERROR_CHECK( mpu_send_byte( REG_INT_EN,     0b00000001, true ) ); // enable data-ready interrupt
    ESP_ERROR_CHECK( mpu_send_byte( REG_PWR_MGMT_1, 0b00001000, true ) ); // disable sleep mode and tmp sensor
    ESP_ERROR_CHECK( mpu_send_byte( REG_PWR_MGMT_2, 0b00000111, true ) ); // disable gyro 
    ESP_ERROR_CHECK( mpu_send_byte( REG_ACCEL_CONF, 0b00011000, true ) ); // config accelerometer sensibility
    ESP_ERROR_CHECK( mpu_send_byte( REG_CONFIG,     0b00000110, true ) ); // config low-pass filter
    ESP_LOGI( I2C_LOGGING_TAG, "%s", "MPU registers configured" );

    #if I2C_TASK_CHECK_REGS == 1
        /* Print all MPU registers */
        ESP_ERROR_CHECK( mpu_check_reg_values() );
    #endif

    /* Task creation */
	xTaskCreate( 	prvI2CTask, "I2CTask", 
					I2C_TASK_STACK_SIZE, NULL, 
					I2C_TASK_PRIORITY, NULL );   
    ESP_LOGI( I2C_LOGGING_TAG, "Task created" );    
}

void 
prvI2CTask( void *pvParameters )
{  
    #if I2C_TASK_WATERMARK == 1
        UBaseType_t uxHighWaterMark;
    #endif

    vector< string > reg_names = { "AX_L", "AX_H", "AY_L", "AY_H", "AZ_L", "AZ_H" };
    vector< string > dir_names = { "AX", "AY", "AZ" };
    vector< uint8_t > reg_values = { REG_AX_L, REG_AX_H, REG_AY_L, REG_AY_H, REG_AZ_L, REG_AZ_H };
    
    uint8_t *received_values_8bits = (uint8_t*)malloc( reg_values.size() * sizeof(uint8_t) );
    uint16_t **received_values_16bits = (uint16_t**)malloc( (reg_values.size() / 2) * sizeof(uint16_t*) );

    for(;;) 
	{     
        uint8_t i;

        /* Initalization of arrays */
        for( i = 0; i < reg_values.size(); i++ )
        {
            received_values_8bits[ i ] = 0;
            if( i % 2 == 0)
            {
                received_values_16bits[ i / 2 ] = 0;
            }
        }


        /* Data request and reception */
        for( i = 0; i < reg_values.size(); i ++ )
        {
            ESP_LOGI( I2C_LOGGING_TAG, "MPU request register %s (%d)...", reg_names.at( i ).c_str(), reg_values.at( i ) );
            mpu_send_byte( reg_values.at( i ), 0, false );
            mpu_receive_byte( received_values_8bits + i * sizeof(uint8_t) );
        }

        /* Parse and log received data in 16 bits */
        received_values_16bits[ 0 ] = (uint16_t*) ( received_values_8bits ); 
        received_values_16bits[ 1 ] = (uint16_t*) ( received_values_8bits + sizeof(uint16_t) * 1 );
        received_values_16bits[ 2 ] = (uint16_t*) ( received_values_8bits + sizeof(uint16_t) * 2 );
        string values_log = "Received values:\n";
        for( i = 0; i < reg_values.size() / 2; i ++ )
        {
            values_log += "\t\t\t";
            values_log += string( dir_names.at( i ) );
            values_log += " = ";
            values_log += to_string( *received_values_16bits[ i ] );
            if( i < reg_values.size() / 2 - 1 )
            {
                values_log += "\n";
            }
        }
        ESP_LOGI( I2C_LOGGING_TAG, "%s", values_log.c_str() );

        /* Log watermark task if desired */
        #if I2C_TASK_WATERMARK == 1
            uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
            ESP_LOGI( I2C_LOGGING_TAG, "Task WaterMark: %d", uxHighWaterMark );
        #endif

        /* Sleep */
        vTaskDelay( I2C_DELAY_MS / portTICK_PERIOD_MS);
        //ESP_LOGI( I2C_LOGGING_TAG, "%s", "Entering deep-sleep mode..." );
        //start_deep_sleep();
    }

    free( received_values_8bits );
    free( received_values_16bits );
	vTaskDelete( NULL );
}

esp_err_t
i2c_init()
{
    esp_err_t err = ESP_OK;

    /* I2C config */
    i2c_config_t i2c_conf;
    i2c_conf.mode               = I2C_MODE_MASTER;
    i2c_conf.sda_io_num         = GPIO_PIN_18;
    i2c_conf.sda_pullup_en      = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_io_num         = GPIO_PIN_19;
    i2c_conf.scl_pullup_en      = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed   = I2C_MASTER_FREQ_HZ;
    
    err = i2c_param_config( I2C_NUM_0, &i2c_conf );
    if( err != ESP_OK )
        return err;
    else
        return i2c_driver_install( I2C_NUM_0, i2c_conf.mode, 0, 0, 0 );
}

esp_err_t
mpu_receive_byte( uint8_t *value_p ) 
{
    esp_err_t err = ESP_OK;

	// Create I2C command buffer
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Queue start signal
	err = i2c_master_start( cmd );
    if( err != ESP_OK )
    {
        goto esc;
    }

	// Queue the slave address (operation = read)
	err = i2c_master_write_byte( cmd, ( I2C_SLAVE_ADDR << 1 ) | I2C_MASTER_READ, true);
    if( err != ESP_OK )
    {
        goto esc;
    }

	// Read from the bus
	err = i2c_master_read_byte( cmd, value_p, I2C_MASTER_ACK );
    if( err != ESP_OK )
    {
        goto esc;
    }

	// Queue the master stop command
	err = i2c_master_stop( cmd );
    if( err != ESP_OK )
    {
        goto esc;
    }

	// Execute the command queue
	err = i2c_master_cmd_begin(	I2C_NUM_0, cmd,	portMAX_DELAY );
    if( err != ESP_OK )
    {
        goto esc;
    }

    esc:

	// Destroy (recycle) command queue
	i2c_cmd_link_delete( cmd );

	return err;
}

esp_err_t
mpu_send_byte( uint8_t reg, uint8_t value, bool write ) 
{
    esp_err_t err = ESP_OK;

	// Create I2C command buffer
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	// Queue start signal
	err = i2c_master_start( cmd );
    if( err != ESP_OK )
    {
        goto esc;
    }

	// Queue adjusted slave address (operation = write)
	err = i2c_master_write_byte( cmd, ( I2C_SLAVE_ADDR << 1 ) | I2C_MASTER_WRITE, true);
    if( err != ESP_OK )
    {
        goto esc;
    }

	// Queue the register
	err = i2c_master_write_byte( cmd, reg, true );
    if( err != ESP_OK )
    {
        goto esc;
    }

    // Queue data (if write)
	if ( write )
    {
        err = i2c_master_write_byte( cmd, value, true ); 
		if( err != ESP_OK )
        {
            goto esc;
        }
	} 

	// Queue stop command
	err = i2c_master_stop( cmd );
    if( err != ESP_OK )
    {
        goto esc;
    }

	// Execute the command queue
	err = i2c_master_cmd_begin( I2C_NUM_0, cmd, portTICK_PERIOD_MS );
    if( err != ESP_OK )
    {
        goto esc;
    }

    esc:

	// Destroy (recycle) command queue
	i2c_cmd_link_delete( cmd );

    return err;
}

void
start_deep_sleep()
{
    ESP_ERROR_CHECK( rtc_gpio_pulldown_en( GPIO_PIN_2 ) );
    ESP_ERROR_CHECK( esp_sleep_enable_ext0_wakeup( GPIO_PIN_2, 1 ) );
	esp_deep_sleep_start();
}

esp_err_t
nvs_read( string storage, string key, uint64_t *value )
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
nvs_write( string storage, string key, uint64_t value )
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

esp_err_t
mpu_check_reg_values()
{
    uint8_t value = 0;
    uint8_t reg;
    esp_err_t err = ESP_OK;

    for( reg = 13; reg < 118; reg++ )
    {
        err = mpu_send_byte( reg, 0, false );
        if( err != ESP_OK )
            return err;
        mpu_receive_byte( &value );
        ESP_LOGI( I2C_LOGGING_TAG, "[%d] = %d", reg, value );
    }

    return err;
}