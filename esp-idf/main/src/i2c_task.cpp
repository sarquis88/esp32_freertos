#include "../include/i2c_task.h"


static uint8_t buffer[ I2C_MASTER_BUFFER_SIZE ];

void 
start_i2c_task(void)
{
    /* I2C config */
    ESP_ERROR_CHECK( i2c_master_init() );
    ESP_ERROR_CHECK( i2c_slave_init() );
    ESP_LOGI( I2C_LOGGING_TAG, "I2C configured" );

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

    // uint8_t *data = (uint8_t*)malloc( sizeof(uint8_t) * BUFFER_SIZE );
    uint8_t data[ I2C_SLAVE_BUFFER_SIZE ];

    sprintf( (char*)data, "%s", "HELLO PAPILO" );

    for(;;) 
	{   
        ESP_LOGI( I2C_LOGGING_TAG, "Slave -> Sending..." );
        i2c_slave_write_buffer( I2C_NUM_1, data, I2C_SLAVE_BUFFER_SIZE, I2C_DELAY_MS / portTICK_PERIOD_MS );
        ESP_LOGI( I2C_LOGGING_TAG, "Slave -> Sended" );

        vTaskDelay( I2C_DELAY_MS / portTICK_PERIOD_MS );
        
        ESP_LOGI( I2C_LOGGING_TAG, "Master: Receiving..." );
        memset( buffer, 0, I2C_MASTER_BUFFER_SIZE );
        i2c_master_read_slave( I2C_NUM_0, buffer, I2C_MASTER_BUFFER_SIZE );
        ESP_LOGI( I2C_LOGGING_TAG, "Master: Data received: %s", (char*)buffer );

        vTaskDelay( I2C_DELAY_MS / portTICK_PERIOD_MS );
        
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
i2c_slave_init()
{
    /* I2C config */
    i2c_config_t i2c_conf_slave;
    i2c_conf_slave.mode                 = I2C_MODE_SLAVE;
    i2c_conf_slave.sda_io_num           = GPIO_PIN_25;
    i2c_conf_slave.sda_pullup_en        = GPIO_PULLUP_ENABLE;
    i2c_conf_slave.scl_io_num           = GPIO_PIN_33;
    i2c_conf_slave.scl_pullup_en        = GPIO_PULLUP_ENABLE;
    i2c_conf_slave.slave.addr_10bit_en  = 0;
    i2c_conf_slave.slave.slave_addr     = I2C_SLAVE_ADDR;
    i2c_param_config( I2C_NUM_1, &i2c_conf_slave );

    return i2c_driver_install( I2C_NUM_1, i2c_conf_slave.mode, I2C_SLAVE_BUFFER_SIZE, I2C_SLAVE_BUFFER_SIZE, 0 );
}

esp_err_t
i2c_master_read_slave( i2c_port_t i2c_num, uint8_t *buffer, size_t size ) 
{
    /* Master reading */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte( cmd, (I2C_SLAVE_ADDR << 1) | READ_BIT, true );
    i2c_master_read( cmd, buffer, size - 1, I2C_MASTER_ACK );
    i2c_master_read_byte( cmd, buffer + size - 1, I2C_MASTER_NACK );
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