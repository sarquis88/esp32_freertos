// TODO: stub function
// TODO: slow i2c freq

#include "../include/accelerometer_task.h"

/* Static variables declarations */
static MPU6050 mpu;
static I2Cdev i2cdev;

/* ######################################################################### */
/* ######################################################################### */

void 
start_accelerometer_task( void )
{
    /* I2C init */
    i2cdev = I2Cdev();
    ESP_ERROR_CHECK( i2c_init() );
    #if ACCELEROMETER_TASK_LOGGING == 1
    ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "I2C initialized" );
    #endif

    /* MPU init */
    mpu = MPU6050();
    mpu_init();
    #if ACCELEROMETER_TASK_LOGGING == 1
    ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "MPU initialized" );
    #endif

    /* Checking FIFO count */
    if( mpu.getFIFOCount() < MPU_FIFO_SIZE )
    {
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Deep-sleep mode on" );
        start_deep_sleep_mode();
    }

    /* NVS init */
    ESP_ERROR_CHECK( nvs_init() );
    #if ACCELEROMETER_TASK_LOGGING == 1
    ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "NVS initialized" );
    #endif

    /* Task creation */
	xTaskCreate( 	prvAccelerometerTask, "accelerometer", 
					ACCELEROMETER_TASK_STACK_SIZE, NULL, 
					ACCELEROMETER_TASK_PRIORITY, NULL );  
    #if ACCELEROMETER_TASK_LOGGING == 1 
    ESP_LOGI( ACCELEROMETER_TASK_TAG, "Task initialized" );    
    #endif
}

void 
prvAccelerometerTask( void *pvParameters )
{  
    /* Variables declaration */
    int16_t *mpu_accel_values;
    uint16_t *mpu_module_array;

    /* Variables initialization */
    mpu_accel_values = ( int16_t*  ) malloc ( MPU_AXIS_COUNT * sizeof( int16_t ) );
    mpu_module_array = ( uint16_t* ) malloc ( MPU_GROUP_SIZE * sizeof( uint16_t ) );

    /* Start task loop */
    for(;;) 
	{    
        /* Variables declaration */
        uint16_t i, fifo_count;
        #if ACCELEROMETER_TASK_LOGGING == 1
        fort::char_table table;
        #endif
        
        /* Variables initialization */
        i = 0;
        #if ACCELEROMETER_TASK_LOGGING == 1
        table << fort::header << "N" << "Ax" << "Ay" << "Az" << "A" << "FIFO" << fort::endr;
        #endif

        /* Start MPU data read */
        for( i = 0; i < MPU_GROUP_SIZE; i++ ) 
        {
            /* Variables declaration */
            uint8_t j;
            uint32_t module;

            /* Data request and reception */
            mpu.getFIFOBytes( (uint8_t*) mpu_accel_values, MPU_AXIS_COUNT * 2 );
    
            /* Swap nibbles */
            for( j = 0; j < MPU_AXIS_COUNT; j++ )
            {   
                uint8_t h_byte = ( mpu_accel_values[ j ] & 0xFF00 ) >> 8;
                uint8_t l_byte = mpu_accel_values[ j ] & 0x00FF;
                mpu_accel_values[ j ] = l_byte << 8 | h_byte;
            }

            /* Calculate data module */
            module = 0;
            for( j = 0; j < MPU_AXIS_COUNT; j++ )
                module += (uint32_t) mpu_accel_values[ j ] * mpu_accel_values[ j ];
            module = (uint32_t) sqrt( (double)module );

            /* Module convertion to 16-bit */
            mpu_module_array[ i ] = atoi( to_string( module ).c_str() );

            /* Updating FIFO count */
            fifo_count = mpu.getFIFOCount();
            
            /* Concat log data */
            #if ACCELEROMETER_TASK_LOGGING == 1
            table << i + 1;
            for( j = 0; j < MPU_AXIS_COUNT; j++ )
                table << mpu_accel_values[ j ];
            table << module << fifo_count << fort::endr;
            #endif
        }

        /* Store data into NVS */
        ESP_ERROR_CHECK( nvs_write( mpu_module_array, MPU_GROUP_SIZE * 2 ) );

        /* Log data */
        #if ACCELEROMETER_TASK_LOGGING == 1
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "MPU data log\n\n%s", table.to_string().c_str() );
        #endif

        /* Reset FIFO */
        mpu.setFIFOEnabled( false );
        mpu.resetFIFO();

        /* Sleep */
        #if ACCELEROMETER_TASK_LOGGING == 1
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Deep-sleep mode on" );
        #endif
        start_deep_sleep_mode();
    }

    /* Task should not reach here */
    #if ACCELEROMETER_TASK_LOGGING == 1
    ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Task ended" );
    #endif
    free( mpu_accel_values );
    free( mpu_module_array );
	vTaskDelete( NULL );
}

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

void
mpu_init()
{   
    mpu.setIntFIFOBufferOverflowEnabled( true );

    mpu.setInterruptLatchClear( true );

    mpu.setStandbyXGyroEnabled( true );
    mpu.setStandbyYGyroEnabled( true );
    mpu.setStandbyZGyroEnabled( true );

    mpu.setFullScaleAccelRange( 0x00 );

    mpu.setDLPFMode( MPU6050_DLPF_BW_5 );

    mpu.setRate( 0xFF );

    mpu.setFIFOEnabled( true );

    mpu.setAccelFIFOEnabled( true );

    mpu.setSleepEnabled( false );
    mpu.setTempSensorEnabled( false );
}

void
mpu_check_reg_values()
{
    uint8_t value = 0;
    uint8_t reg;

    for( reg = 13; reg < 118; reg++ )
    {
        if( i2cdev.readByte( I2C_SLAVE_ADDR, reg, &value ) == true )
            ESP_LOGI( ACCELEROMETER_TASK_TAG, "[%d] = %d", reg, value );
        else
            ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Cannot check registers values" );
    }
}

/* ######################################################################### */
/* ######################################################################### */

esp_err_t
nvs_init()
{
    esp_err_t err = nvs_flash_init();
    if ( err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND ) 
    {
        #if ACCELEROMETER_TASK_LOGGING == 1
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "NVS partition was truncated and needs to be erased. Retrying..." );
        #endif
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }

    return err;   
}

esp_err_t
nvs_read( uint16_t* data, size_t* len )
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open( NVS_STORAGE_NAME, NVS_READWRITE, &my_handle );

    if ( err == ESP_OK ) 
    {
        err = nvs_get_blob( my_handle, NVS_KEY_NAME, (void*)data, len );
        
        if( err == ESP_ERR_NVS_NOT_FOUND )
        {
            *data = 0;
            err = ESP_OK;
        }
        nvs_close( my_handle );
    }
    
    return err;
}

esp_err_t
nvs_write( uint16_t* data, size_t len )
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open( NVS_STORAGE_NAME, NVS_READWRITE, &my_handle );

    if ( err == ESP_OK ) 
    {
        err = nvs_set_blob( my_handle, NVS_KEY_NAME, (void*)data, len );
        if( err == ESP_OK )
        {
            err = nvs_commit( my_handle );
        }
        nvs_close( my_handle );
    }
    
    return err;
}

/* ######################################################################### */
/* ######################################################################### */

void
start_deep_sleep_mode()
{
    ESP_ERROR_CHECK( rtc_gpio_pulldown_en( GPIO_PIN_2 ) );
    ESP_ERROR_CHECK( esp_sleep_enable_ext0_wakeup( GPIO_PIN_2, 1 ) );
	esp_deep_sleep_start();
}
