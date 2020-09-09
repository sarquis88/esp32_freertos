// TODO: stub function
// TODO: binary image header

#include "../include/accelerometer_task.h"

/* Static variables declarations */
static MPU6050 mpu;
static I2Cdev i2cdev;

/* RTC variables declarations */
RTC_DATA_ATTR half rtc_mpu_data_array[ 1 ];
RTC_DATA_ATTR uint16_t rtc_mpu_data_index = 0;

/* Precompilation definitions */
#define ACCELEROMETER_TASK_VERBOSITY_LEVEL      ( 3 )
#define ACCELEROMETER_TASK_SHOW_WATERMARK       ( 0 )

/* ######################################################################### */
/* ######################################################################### */

void 
start_accelerometer_task( void )
{
    /* I2C init */
    i2cdev = I2Cdev();
    ESP_ERROR_CHECK( i2c_init() );
    #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
    ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "I2C initialized" );
    #endif

    /* MPU init */
    mpu = MPU6050();
    mpu_init();
    #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
    ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "MPU initialized" );
    #endif

    /* Checking FIFO count */
    if( mpu.getFIFOCount() < MPU_FIFO_SIZE )
    {
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Deep-sleep mode on" );
        start_deep_sleep_mode();
    }

    /* Task creation */
	xTaskCreate( 	prvAccelerometerTask, "accelerometer", 
					ACCELEROMETER_TASK_STACK_SIZE, NULL, 
					ACCELEROMETER_TASK_PRIORITY, NULL );  
    #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0 
    ESP_LOGI( ACCELEROMETER_TASK_TAG, "Task initialized" );    
    #endif
}

void 
prvAccelerometerTask( void *pvParameters )
{  
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

        /* Start MPU data read */
        for( i = 0; i < MPU_GROUP_SIZE; i++ ) 
        {
            /* RTC RAM storage check */
            if( rtc_mpu_data_index >= RTC_MPU_DATA_SIZE )
            {
                /* Log RTC storage values */ 
                #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 1
                check_rtc_values( 0, rtc_mpu_data_index );
                #endif

                /* Send data through WiFi */
                #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
                ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "RTC storage is full. Sending data through WiFi" );
                #endif
                send_data();

                /* Clearing RTC storage for using it again */ 
                #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
                ESP_LOGI( ACCELEROMETER_TASK_TAG, "%s", "Clearing RTC storage" );
                #endif
                clear_rtc_storage();
            }

            /* Variables declaration */
            uint8_t j;
            half module_half;
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
            
            /* Module normalization */
            module = module / MPU_SENSITIVITY;

            /* Module to half float */
            module_half = module;

            /* Module convertion to 16-bit and store into RTC RAM */
            rtc_mpu_data_array[ rtc_mpu_data_index ] = module_half;
            rtc_mpu_data_index++;
        }

        /* Log data */ 
        #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 0
        #if ACCELEROMETER_TASK_VERBOSITY_LEVEL > 2
        check_rtc_values( rtc_index_first, rtc_mpu_data_index);
        #endif
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "Data stored in RTC from index %d to %d", rtc_index_first, rtc_mpu_data_index );
        #endif

        /* Reset FIFO */
        mpu.setFIFOEnabled( false );
        mpu.resetFIFO();

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

void
start_deep_sleep_mode()
{
    ESP_ERROR_CHECK( rtc_gpio_pulldown_en( GPIO_PIN_2 ) );
    ESP_ERROR_CHECK( esp_sleep_enable_ext0_wakeup( GPIO_PIN_2, 1 ) );
	esp_deep_sleep_start();
}

void
check_rtc_values( uint16_t inf_limit, uint16_t sup_limit)
{
    if( rtc_mpu_data_index >= inf_limit && rtc_mpu_data_index <= sup_limit )
    {
        uint16_t i;
        float avg;

        avg = 0;

        for( i = 0; i < rtc_mpu_data_index; i++ )
        {
            avg += ( float ) rtc_mpu_data_array[ i ];

            ESP_LOGI( ACCELEROMETER_TASK_TAG, "[%d] %f g", i, ( float ) rtc_mpu_data_array[ i ] );
        }

        avg = avg / rtc_mpu_data_index;

        ESP_LOGI( ACCELEROMETER_TASK_TAG, "[AVG] %f g", avg );   
    }
    else
    {
        ESP_LOGI( ACCELEROMETER_TASK_TAG, "Nothing into the RTC limits [%d, %d]", inf_limit, sup_limit );
    }
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
send_data()
{
    return;
}