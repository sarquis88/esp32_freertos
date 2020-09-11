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
    /* Variables declaration */
    int16_t mpu_accel_values[ MPU_AXIS_COUNT ];
    uint32_t buffer;

    /* Start task loop */
    for(;;) 
	{    
        if( xQueueReceive( interrupt_queue, &buffer, portMAX_DELAY ) ) 
		{
			/* Variables declaration */
            uint8_t j;
            float module;

            /* Data request and reception */
            mpu_accel_values[ 0 ] = mpu.getAccelerationX();
            mpu_accel_values[ 1 ] = mpu.getAccelerationY();
            mpu_accel_values[ 2 ] = mpu.getAccelerationZ();

            /* Calculate data module */
            module = 0;
            for( j = 0; j < MPU_AXIS_COUNT; j++ )
                module += mpu_accel_values[ j ] * mpu_accel_values[ j ];
            module = sqrt( module );

            /* Module normalization */
            module = module / MPU_SENSITIVITY;

            /* Log data */
            #if ACCELEROMETER_TASK_LOGGING == 1
                ESP_LOGI( ACCELEROMETER_TASK_TAG, "MPU -> [%f]", module );
            #endif
        }
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

/* ######################################################################### */
/* ######################################################################### */

void
IRAM_ATTR gpio_int_handler( void* arg )
{
	uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR( interrupt_queue, &gpio_num, NULL);
}