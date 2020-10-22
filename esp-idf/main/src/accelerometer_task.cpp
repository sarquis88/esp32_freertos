#include "../include/accelerometer_task.h"

/* Static variables declarations */
static MPU6050 mpu;
static I2Cdev i2cdev;
static xQueueHandle interrupt_queue = NULL;

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

    /* Interruption queue */
	interrupt_queue = xQueueCreate( 10, sizeof(uint32_t) );

	/* Input config */
	gpio_config_t io_conf;
	io_conf.pull_down_en 	= ( gpio_pulldown_t ) 0;
	io_conf.intr_type 		= GPIO_INTR_ANYEDGE;
    io_conf.mode 			= GPIO_MODE_INPUT;
    io_conf.pin_bit_mask 	= 1ULL<<GPIO_PIN_2;
    io_conf.pull_up_en 		= ( gpio_pullup_t ) 1;
    ESP_ERROR_CHECK( gpio_config( &io_conf ) );
    ESP_ERROR_CHECK( gpio_install_isr_service( 0 ) );
    ESP_ERROR_CHECK( gpio_isr_handler_add( GPIO_PIN_2, gpio_int_handler, (void*) 88 ) );

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
    /* Start task loop */
    for(;;) 
	{    
        int16_t accel_value, offset;
        uint16_t steps[ AUTOCALIBRATION_STEPS ];
        uint8_t step_level, step_level_counter, axis;
        float accel_value_module;

        offset = 0;
        step_level = 0;
        step_level_counter = 0;
        axis = 0;
        steps[ 0 ] = AUTOCALIBRATION_STEP_L0;
        steps[ 1 ] = AUTOCALIBRATION_STEP_L1;
        steps[ 2 ] = AUTOCALIBRATION_STEP_L2;
        steps[ 3 ] = AUTOCALIBRATION_STEP_L3;

        mpu.setXAccelOffset( offset );

        while( true )
        {
            if( axis == 0 )
                accel_value = mpu.getAccelerationX();
            else if( axis == 1 )
                accel_value = mpu.getAccelerationY();
            else if( axis == 2 )
                accel_value = mpu.getAccelerationZ();
            

            ESP_LOGI( ACCELEROMETER_TASK_TAG, "axis=%d, value=%d offset=%d", axis, accel_value, offset );

            accel_value_module = sqrt( accel_value * accel_value );

            if( accel_value_module < 100 )
            {
                if( step_level != 3 )
                {
                    step_level = 3;
                    step_level_counter = 0;
                }
            }
            else if( accel_value_module < 500 )
            {
                if( step_level != 2 )
                {
                    step_level = 2;
                    step_level_counter = 0;
                }
            }
            else if( accel_value_module < 6000 )
            {
                if( step_level != 1 )
                {
                    step_level = 1;
                    step_level_counter = 0;
                }
            }
            else
            {
                if( step_level != 0 )
                {
                    step_level = 0;
                    step_level_counter = 0;
                }
            }

            if( accel_value > 0 )
            {
                offset -= steps[ step_level ];
            }
            else if( accel_value < 0  )
            {
                offset += steps[ step_level ];
            }

            step_level_counter++;

            if( step_level_counter == 15 )
            {
                if( step_level == AUTOCALIBRATION_STEPS - 1 )
                {
                    axis++;
                    step_level_counter = 0;
                }
                else
                {
                    esp_restart();
                }
            }

            if( axis == 0 )
                mpu.setXAccelOffset( offset );
            else if( axis == 1 )
                mpu.setYAccelOffset( offset );
            else if( axis == 2 )
                mpu.setZAccelOffset( offset );
            else 
            {
                #if ACCELEROMETER_TASK_LOGGING == 1
                ESP_LOGI( ACCELEROMETER_TASK_TAG, "C'est fini" );
                #endif

                for(;;)
                {
                    vTaskDelay( ACCELEROMETER_TASK_DELAY_MS / portTICK_PERIOD_MS);
                }
            }

            vTaskDelay( ACCELEROMETER_TASK_DELAY_MS / portTICK_PERIOD_MS);
        }

        /* Log data */
        #if ACCELEROMETER_TASK_LOGGING == 1
            ESP_LOGI( ACCELEROMETER_TASK_TAG, "X=%d", accel_value );
        #endif
    }

    /* Task should not reach here */
    #if ACCELEROMETER_TASK_LOGGING == 1
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
    i2c_conf.sda_io_num         = GPIO_PIN_19;
    i2c_conf.sda_pullup_en      = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_io_num         = GPIO_PIN_18;
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
    mpu.setIntDataReadyEnabled( true);

    mpu.setInterruptLatchClear( true );

    mpu.setStandbyXGyroEnabled( true );
    mpu.setStandbyYGyroEnabled( true );
    mpu.setStandbyZGyroEnabled( true );

    mpu.setFullScaleAccelRange( 0x00 );

    mpu.setDLPFMode( MPU6050_DLPF_BW_5 );

    mpu.setRate( 0xFF );

    mpu.setSleepEnabled( false );
    mpu.setTempSensorEnabled( false );

    
}

void
mpu_calibration()
{
    int16_t accel_value, offset;

    offset = 0;
    
    while( true )
    {
        accel_value = mpu.getAccelerationX();

        ESP_LOGI( ACCELEROMETER_TASK_TAG, "AX=%d, OF=%d", accel_value, offset );

        if( accel_value > 0 )
            offset = offset - 10;
        else if( accel_value < 0 )
            offset = offset + 10;
            
        mpu.setXAccelOffset( offset );

        vTaskDelay( 100 / portTICK_PERIOD_MS);
    }
}

/* ######################################################################### */
/* ######################################################################### */

void
IRAM_ATTR gpio_int_handler( void* arg )
{
	uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR( interrupt_queue, &gpio_num, NULL);
}

/* ######################################################################### */
/* ######################################################################### */

uint8_t
get_scaled_module( int16_t raw_data[ MPU_AXIS_COUNT ] )
{
    uint8_t j;
    float module;

    module = 0;
    for( j = 0; j < MPU_AXIS_COUNT; j++ )
        module += raw_data[ j ] * raw_data[ j ];
    module = sqrt( module );

    return ( module / 257.00 );
}