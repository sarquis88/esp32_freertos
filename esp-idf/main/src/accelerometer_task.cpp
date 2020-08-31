#include "../include/accelerometer_task.h"

void 
start_accelerometer_task( void )
{
    /* NVS init */
    ESP_ERROR_CHECK( nvs_init() );
    #if ACCELEROMETER_TASK_LOGGING == 1
    ESP_LOGI( ACCELEROMETER_LOGGING_TAG, "%s", "NVS initialized" );
    #endif

    /* I2C init */
    ESP_ERROR_CHECK( i2c_init() );
    #if ACCELEROMETER_TASK_LOGGING == 1
    ESP_LOGI( ACCELEROMETER_LOGGING_TAG, "%s", "I2C initialized" );
    #endif

    /* MPU init */
    ESP_ERROR_CHECK( mpu_init() );
    #if ACCELEROMETER_TASK_LOGGING == 1
    ESP_LOGI( ACCELEROMETER_LOGGING_TAG, "%s", "MPU initialized" );
    #endif

    /* Task creation */
	xTaskCreate( 	prvAccelerometerTask, "accelerometer", 
					ACCELEROMETER_TASK_STACK_SIZE, NULL, 
					ACCELEROMETER_TASK_PRIORITY, NULL );  
    #if ACCELEROMETER_TASK_LOGGING == 1 
    ESP_LOGI( ACCELEROMETER_LOGGING_TAG, "Task initialized" );    
    #endif
}

void 
prvAccelerometerTask( void *pvParameters )
{  
    /* Variables declaration */
    vector< string > mpu_value_names;
    vector< uint8_t > mpu_reg_addrs;
    int8_t *accel_values_8b;
    int16_t **accel_values_16b;
    uint16_t j;
    #if ACCELEROMETER_TASK_WATERMARK == 1
        UBaseType_t uxHighWaterMark;
    #endif

    /* Variables initialization */
    j = 0;
    mpu_value_names = { "AX", "AY", "AZ", "A" };
    mpu_reg_addrs = { REG_AX_L, REG_AX_H, REG_AY_L, REG_AY_H, REG_AZ_L, REG_AZ_H };
    accel_values_8b = (int8_t*)malloc( mpu_reg_addrs.size() * sizeof(int8_t) );
    accel_values_16b = (int16_t**)malloc( (mpu_reg_addrs.size() / 2) * sizeof(int16_t*) );

    for(;;) 
	{     
        /* Variables declaration */
        uint8_t i;
        uint64_t module, nvs_index;

        /* Variables initialization */
        for( i = 0; i < mpu_reg_addrs.size(); i++ )
        {
            accel_values_8b[ i ] = 0;
            if( i % 2 == 0)
                accel_values_16b[ i / 2 ] = 0;
        }

        /* NVS index get */
        nvs_index = 0;
        ESP_ERROR_CHECK( nvs_read( NVS_STORAGE_NAME, NVS_INDEX_KEY, &nvs_index ) );

        /* Data request and reception */
        for( i = 0; i < mpu_reg_addrs.size(); i ++ )
        {
            ESP_ERROR_CHECK( mpu_send_byte( mpu_reg_addrs.at( i ), 0, false ) );
            mpu_receive_byte( (uint8_t*)accel_values_8b + i * sizeof(uint8_t) );
        }

        /* Convert 8-bit received data into 16-bit */
        accel_values_16b[ 0 ] = (int16_t*) ( accel_values_8b ); 
        accel_values_16b[ 1 ] = (int16_t*) ( accel_values_8b + sizeof(int16_t) * 1 );
        accel_values_16b[ 2 ] = (int16_t*) ( accel_values_8b + sizeof(int16_t) * 2 );

        /* Calculate data module */
        module = 0;
        for( i = 0; i < mpu_reg_addrs.size() / 2; i ++ )
            module += (uint64_t) pow( (double)*accel_values_16b[ i ], 2 );
        module = (uint64_t) sqrt( (double)module );

        /* Log data */
        #if ACCELEROMETER_TASK_LOGGING == 1
        log_results( j++, accel_values_16b, module );
        #endif

        if( nvs_index < 627 )
        {
            /* Store module into NVS */
            string key = string( NVS_STORAGE_NAME ) + to_string( nvs_index );
            ESP_ERROR_CHECK( nvs_write( NVS_STORAGE_NAME, key.c_str(), module ) );

            /* Update of NVS index */
            nvs_index++;
            ESP_ERROR_CHECK( nvs_write( NVS_STORAGE_NAME, NVS_INDEX_KEY, nvs_index ) );
        }

        /* Log watermark task */
        #if ACCELEROMETER_TASK_WATERMARK == 1 and ACCELEROMETER_TASK_LOGGING == 1
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        ESP_LOGI( ACCELEROMETER_LOGGING_TAG, "Task WaterMark: %d", uxHighWaterMark );
        #endif

        /* Delay */
        vTaskDelay( ACCELEROMETER_TASK_DELAY_MS / portTICK_PERIOD_MS );
    }

    /* Task should not reach here */
    #if ACCELEROMETER_TASK_LOGGING == 1
    ESP_LOGI( ACCELEROMETER_LOGGING_TAG, "%s", "Task ended" );
    #endif
    free( accel_values_8b );
    free( accel_values_16b );
	vTaskDelete( NULL );
}

/* ######################################################################### */
/* ######################################################################### */
/* ######################################################################### */

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

/* ######################################################################### */
/* ######################################################################### */
/* ######################################################################### */

esp_err_t
mpu_init()
{    
    esp_err_t err;

    err = mpu_send_byte( REG_INT_ENABLE,    0b00000001, true ); // enable data-ready interrupt
    if( err != ESP_OK )
        return err;

    err = mpu_send_byte( REG_INT_PIN_CFG,   0b00010000, true ); // clear int when reading
    if( err != ESP_OK )
        return err;

    err = mpu_send_byte( REG_PWR_MGMT_2,    0b00000111, true ); // disable gyro 
    if( err != ESP_OK )
        return err;

    err = mpu_send_byte( REG_ACCEL_CONF,    0b00000000, true ); // config accelerometer sensibility
    if( err != ESP_OK )
        return err;

    err = mpu_send_byte( REG_CONFIG,        0b00000110, true ); // config low-pass filter
    if( err != ESP_OK )
        return err;

    err = mpu_send_byte( REG_SMPRT_DIV,     0b11111111, true ); // config sample-rate divider
    if( err != ESP_OK )
        return err;

    err = mpu_send_byte( REG_PWR_MGMT_1,    0b00001000, true ); // disable sleep mode and tmp sensor
    if( err != ESP_OK )
        return err;

    return err;
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
	err = i2c_master_cmd_begin( I2C_NUM_0, cmd, portMAX_DELAY );
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
        ESP_LOGI( ACCELEROMETER_LOGGING_TAG, "[%d] = %d", reg, value );
    }

    return err;
}

/* ######################################################################### */
/* ######################################################################### */
/* ######################################################################### */

esp_err_t
nvs_init()
{
    esp_err_t err = nvs_flash_init();
    if ( err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND ) 
    {
        #if ACCELEROMETER_TASK_LOGGING == 1
        ESP_LOGI( ACCELEROMETER_LOGGING_TAG, "%s", "NVS partition was truncated and needs to be erased. Retrying..." );
        #endif
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }

    return err;   
}

esp_err_t
nvs_read( const char* storage, const char* key, uint64_t *value )
{
    /* Open nvs */
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open( storage, NVS_READWRITE, &my_handle );
    if ( err == ESP_OK ) 
    {
        /* Read value */
        err = nvs_get_u64( my_handle, key, value );
        nvs_close( my_handle );
    }
    
    return err;
}

esp_err_t
nvs_write( const char* storage, const char* key, uint64_t value )
{
    /* Open nvs */
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open( storage, NVS_READWRITE, &my_handle );
    if ( err == ESP_OK ) 
    {
        /* Write value */
        err = nvs_set_u64( my_handle, key, value );
        if( err == ESP_OK )
        {
            err = nvs_commit( my_handle );
        }
        nvs_close( my_handle );
    }

    return err;
}

esp_err_t
nvs_check_values()
{
    string log;
    char key[ BUFFER_SIZE ];
    uint16_t i;
    uint64_t nvs_index, read;
    float average = 0;
    
    nvs_index = 0;

    nvs_read( NVS_STORAGE_NAME, NVS_INDEX_KEY, (uint64_t*)&nvs_index );

    if( nvs_index == 0 )
        return ESP_OK;
    
    for( i = 0; i < nvs_index; i++ )
    {
        sprintf( key, "%s%d", NVS_STORAGE_NAME, i );
        nvs_read( NVS_STORAGE_NAME, key, &read );
        ESP_LOGI( ACCELEROMETER_LOGGING_TAG, "NVS %d: %llu", i, read );
        average += read;
    }

    average = average / nvs_index;
    ESP_LOGI( ACCELEROMETER_LOGGING_TAG, "NVS average: %0.f", average );

    return ESP_OK;
}

/* ######################################################################### */
/* ######################################################################### */
/* ######################################################################### */

void
start_deep_sleep_mode()
{
    ESP_ERROR_CHECK( rtc_gpio_pulldown_en( GPIO_PIN_2 ) );
    ESP_ERROR_CHECK( esp_sleep_enable_ext0_wakeup( GPIO_PIN_2, 1 ) );
	esp_deep_sleep_start();
}
 
void
log_results( uint16_t c, int16_t** results_vector, uint64_t module )
{
    vector< string > mpu_value_names = { "AX", "AY", "AZ", "A" };
    size_t result_len = 9;
    size_t i;    
    string log = "[" + to_string( c ) + "] [ ";

    for( i = 0; i < 3; i++ )
    {
        int16_t result = *results_vector[ i ];
        
        string partial_log = mpu_value_names.at( i ) + string( "=" ) + to_string( result );

        uint8_t diff = result_len - partial_log.size();

        uint8_t j;
        for( j = 0; j < diff; j++ )
            partial_log += string( " " );

        partial_log += string( " | " );

        log += partial_log;
    }

    log += string( mpu_value_names.at( 3 ) + string( "=" ) + to_string( module ) ) + " ]";

    ESP_LOGI( ACCELEROMETER_LOGGING_TAG, "%s", log.c_str() );
}