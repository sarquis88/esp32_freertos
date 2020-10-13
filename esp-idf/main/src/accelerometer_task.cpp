#include "../include/accelerometer_task.h"

/* Static variables declarations */
static MPU6050 mpu;
static I2Cdev i2cdev;
static xQueueHandle* accelerometer_task_queue;
static xQueueHandle* transfer_task_queue;

/* RTC variables declarations */
RTC_DATA_ATTR uint8_t rtc_mpu_data_array[ 1 ];
RTC_DATA_ATTR uint16_t rtc_mpu_data_index = 0;
RTC_DATA_ATTR uint32_t spiffs_data_index = 0;
RTC_DATA_ATTR bool first_boot = true;

/* Precompilation definitions */
#if GLOBAL_VERBOSITY_LEVEL > 0
#define ACCELEROMETER_TASK_VERBOSITY_LEVEL      ( 1 )
#endif

void 
start_accelerometer_task( xQueueHandle* reception, xQueueHandle* sending )
{    
    /* Asign passed queues to local ones */
    accelerometer_task_queue = reception;
    transfer_task_queue = sending;

    /* Task creation */
	xTaskCreate( 	prvAccelerometerTask, "accelerometer", 
					ACCELEROMETER_TASK_STACK_SIZE, NULL, 
					ACCELEROMETER_TASK_PRIORITY, NULL );  
}

void 
prvAccelerometerTask( void *pvParameters )
{  
    /* Log tag initialization */
    #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0 
    ESP_LOGI( ACCELEROMETER_TASK_TAG, "Task initialized" );    
    #endif

    /* I2C init */
    i2cdev = I2Cdev();
    ESP_ERROR_CHECK( i2c_init() );
    #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
    ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "I2C initialized" );
    #endif

    /* MPU init */
    mpu = MPU6050();
    if( first_boot )
    {
        first_boot = false;
        mpu_init();
        #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "MPU initialized" );
        #endif

        #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Deep-sleep mode on" );
        #endif
        start_deep_sleep_mode();
    }

    for(;;) 
	{
        /* Variables declaration */
        uint16_t i;
        int16_t mpu_accel_values[ MPU_AXIS_COUNT ];
        
        /* Variables initialization */
        i = 0;
        #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Receiving data" );
        #endif

        /* Start MPU data reading */
        for( i = 0; i < MPU_GROUP_SIZE; i++ ) 
        {
            /* Checking if RTC storage is full */
            if( rtc_mpu_data_index >= RTC_MPU_DATA_SIZE )
            {
                #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
                ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "RAM full" );
                #endif

                /* Writing data to SPIFFS */
                ESP_ERROR_CHECK( write_to_spiffs( rtc_mpu_data_array, rtc_mpu_data_index ) );
                spiffs_data_index += rtc_mpu_data_index;
                #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
                ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Writing data to SPIFFS" );
                #endif

                /* Checking SPIFFS space */
                if( spiffs_data_index >= SPIFFS_BYTES_SIZE )
                {
                    #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
                    ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "SPIFFS full" );
                    #endif
                    send_data_and_wait();
                    remove( SPIFFS_FILE_NAME );
                    spiffs_data_index = 0;
                }

                rtc_mpu_data_index = 0;
            }

            /* Variables declaration */
            uint8_t j, scaled_module;
            float module;

            /* Accelerometer's data request */
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
                module += mpu_accel_values[ j ] * mpu_accel_values[ j ];
            module = sqrt( module );

            /* Scaling module (converting it to 8-bit) */
            scaled_module = ( module / 257.00 );

            /* Storing module into RAM */
            rtc_mpu_data_array[ rtc_mpu_data_index ] = scaled_module;
            #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 2
            ESP_LOGI( ACCELEROMETER_TASK_TAG, "[%d] %d", rtc_mpu_data_index, rtc_mpu_data_array[ rtc_mpu_data_index ] );
            #endif
            rtc_mpu_data_index++;
        }

        /* Log data */ 
        #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "Writing data to RAM" );
        #endif

        /* Sleeping until FIFO overflows */
        #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Deep-sleep mode on" );
        #endif
        start_deep_sleep_mode();
    }

    /* Exit routine. Task should not reach here */
    #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
    ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Task ended" );
    #endif
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
    /* Enable interrupt when MPU's FIFO is full */
    mpu.setIntFIFOBufferOverflowEnabled( true );

    /* Clear interrupt when reading */
    mpu.setInterruptLatchClear( true );

    /* Disabling gyroscope and temp. sensor */
    mpu.setStandbyXGyroEnabled( true );
    mpu.setStandbyYGyroEnabled( true );
    mpu.setStandbyZGyroEnabled( true );
    mpu.setTempSensorEnabled( false );

    /* Calibrating MPU (for 2g sensitivity) */
    mpu.setXAccelOffset( MPU_AX_OFFSET );
    mpu.setYAccelOffset( MPU_AY_OFFSET );
    mpu.setZAccelOffset( MPU_AZ_OFFSET );

    /* Setting accelerometer fullrange (and sensitivity) */
    mpu.setFullScaleAccelRange( 0x00 ); 

    /* Configuring digital low pass filter */
    mpu.setDLPFMode( MPU6050_DLPF_BW_5 );

    /* Setting accelerometer rate */
    mpu.setRate( 0xFF );

    /* Enabling MPU's FIFO */
    mpu.setFIFOEnabled( true );
    mpu.setAccelFIFOEnabled( true );    
    
    /* Starting MPU */
    mpu.setSleepEnabled( false );
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

/* ######################################################################### */
/* ######################################################################### */

void
send_data_and_wait()
{
    uint32_t queue_buffer;

    /* Send start message */
    queue_buffer    = CODE_STARTTRANSFER;
    xQueueSend      ( *transfer_task_queue,   &queue_buffer, portMAX_DELAY );
    xQueueReceive   ( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );

    /* Send data size */
    queue_buffer    = SPIFFS_BYTES_SIZE;
    xQueueSend      ( *transfer_task_queue,   &queue_buffer, portMAX_DELAY );
    xQueueReceive   ( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );

    /* Wait until wifi-transfer is done */
    do
    {
        xQueueReceive   ( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );
    } while( queue_buffer != CODE_ENDTRANSFER );
}

/* ######################################################################### */
/* ######################################################################### */

esp_err_t
write_to_spiffs( uint8_t* data, size_t len )
{
	/* Variables declaration */
	FILE* file;
    esp_err_t err;

    /* Mount SPIFFS partition */
	esp_vfs_spiffs_conf_t conf = 
	{
		.base_path = SPIFFS_BASE_PATH,
		.partition_label = NULL,
		.max_files = 5,
		.format_if_mount_failed = true
	};
	err = esp_vfs_spiffs_register( &conf );
	if( err != ESP_OK && err != ESP_ERR_INVALID_STATE )
    {
		return err;
    }
	
	/* Open accel data file */
    file = fopen( SPIFFS_FILE_NAME, "a+" );
    if (file == NULL) 
	{
		#if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
        ESP_LOGE( MAIN_TASK_TAG, "Failed to open file for writing" );
		#endif
        return ESP_FAIL;
    }

	/* File writing */
    if( fwrite( (void*) data, sizeof( uint8_t ), len, file ) != len )
        return ESP_FAIL;

	/* File closing */
    fclose( file );
    return ESP_OK;
}