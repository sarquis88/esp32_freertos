/*
    TODO
        - I2C in sleep mode
        - Auto calibration
        - Wifi transmission
*/

/*
    ESP32 wakes-up each ~45 seconds, storing 170 accelerometer modules (8-bit each one)
    The maximum storage is 7990 modules, so in ~35 minutes the memory will be full
*/

#include "../include/accelerometer_task.h"

/* Static variables declarations */
static MPU6050 mpu;
static I2Cdev i2cdev;
static xQueueHandle* accelerometer_task_queue;
static xQueueHandle* transfer_task_queue;

/* RTC variables declarations */
RTC_DATA_ATTR uint8_t rtc_mpu_data_array[ 1 ];
RTC_DATA_ATTR uint16_t rtc_mpu_data_index = 0;
RTC_DATA_ATTR bool first_boot = true;

/* Precompilation definitions */
#define ACCELEROMETER_TASK_VERBOSITY_LEVEL      ( 1 )
#define ACCELEROMETER_TASK_SHOW_WATERMARK       ( 0 )

/* ######################################################################### */
/* ######################################################################### */

void 
start_accelerometer_task( xQueueHandle* reception, xQueueHandle* sending )
{    
    /* Asign passed queue to local one */
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
    }

    /* Checking FIFO count */
    if( mpu.getFIFOCount() < MPU_FIFO_SIZE )
    {
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Deep-sleep mode on" );
        start_deep_sleep_mode();
    }

    for(;;) 
	{
        /* Variables declaration */
        uint16_t i;
        int16_t mpu_accel_values[ MPU_AXIS_COUNT ];
        #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
        uint16_t rtc_index_first;
        #endif
        
        /* Variables initialization */
        i = 0;
        #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
        rtc_index_first = rtc_mpu_data_index;
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Receiving data" );
        #endif

        /* Start MPU data reading */
        for( i = 0; i < MPU_GROUP_SIZE; i++ ) 
        {
            /* Checking if RTC storage is full */
            if( rtc_mpu_data_index >= RTC_MPU_DATA_SIZE )
            {
                /* Send data through WiFi */
                #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
                ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "RAM full, sending data to Transfer task" );
                #endif
                send_data_and_wait();

                /* Clearing RTC storage for using it again */ 
                #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
                ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Clearing RAM" );
                #endif
                clear_rtc_storage();
            }

            /* Variables declaration */
            uint8_t j, scaled_module;
            float module;

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
                module += mpu_accel_values[ j ] * mpu_accel_values[ j ];
            module = sqrt( module );

            /* Scaling module */
            scaled_module = ( module / 257.00 );

            /* Module store into RTC RAM */
            rtc_mpu_data_array[ rtc_mpu_data_index ] = scaled_module;
            #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 2
            ESP_LOGI( ACCELEROMETER_TASK_TAG, "[%d] %d", rtc_mpu_data_index, rtc_mpu_data_array[ rtc_mpu_data_index ] );
            #endif
            rtc_mpu_data_index++;
        }

        /* Log data */ 
        #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "Data stored in RTC from index %d to %d", rtc_index_first, rtc_mpu_data_index - 1 );
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

void
start_deep_sleep_mode()
{
    ESP_ERROR_CHECK( rtc_gpio_pulldown_en( GPIO_PIN_2 ) );
    ESP_ERROR_CHECK( esp_sleep_enable_ext0_wakeup( GPIO_PIN_2, 1 ) );
	esp_deep_sleep_start();
}

void
clear_rtc_storage()
{
    uint16_t i;

    #pragma GCC diagnostic ignored "-Waggressive-loop-optimizations"

    for( i = 0; i < RTC_MPU_DATA_SIZE; i++ )
        rtc_mpu_data_array[ i ] = 0.f;

    #pragma GCC diagnostic pop

    rtc_mpu_data_index = 0;
}

void
send_data_and_wait()
{
    uint16_t queue_buffer;
    uint16_t i;

    /* Send start message */
    queue_buffer    = CODE_STARTTRANSFER;
    xQueueSend      ( *transfer_task_queue,   &queue_buffer, portMAX_DELAY );
    xQueueReceive   ( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );

    /* Send data size message */
    queue_buffer    = RTC_MPU_DATA_SIZE;
    xQueueSend      ( *transfer_task_queue,   &queue_buffer, portMAX_DELAY );
    xQueueReceive   ( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );

    /* Send data */
    #pragma GCC diagnostic ignored "-Waggressive-loop-optimizations"
    for( i = 0; i < RTC_MPU_DATA_SIZE; i++ )
    {
        queue_buffer    = rtc_mpu_data_array[ i ];
        xQueueSend      ( *transfer_task_queue,   &queue_buffer, portMAX_DELAY );
        xQueueReceive   ( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );
    }
    #pragma GCC diagnostic pop

    /* Wait until wifi-transfer is done */
    do
    {
        xQueueReceive   ( *accelerometer_task_queue, &queue_buffer, portMAX_DELAY );
    } while( queue_buffer != CODE_ENDTRANSFER );
}