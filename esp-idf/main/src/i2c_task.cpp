#include "../include/i2c_task.h"

void 
start_i2c_task(void)
{

    ESP_ERROR_CHECK( i2c_master_init() );

    /* Task creation */
	xTaskCreate( 	prvI2CTask, "I2CTask", 
					I2C_TASK_STACK_SIZE, NULL, 
					I2C_TASK_PRIORITY, NULL );       
}

void 
prvI2CTask( void *pvParameters )
{   
    uint8_t *buffer = (uint8_t*)malloc( sizeof(uint8_t) * BUFFER_SIZE );

    i2c_master_init();

    for(;;) 
	{   
        ESP_LOGI( I2C_LOGGING_TAG, "Reading from I2C..." );

        //ESP_ERROR_CHECK( i2c_master_read_slave( I2C_NUM_0, buffer, BUFFER_SIZE ) );

        vTaskDelay( 1000 / portTICK_PERIOD_MS);
    }

	vTaskDelete( NULL );
}

esp_err_t
i2c_master_init()
{
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
i2c_master_read_slave( i2c_port_t i2c_num, uint8_t *buffer, size_t size ) 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);

    i2c_master_write_byte( cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, true );
    
    i2c_master_read( cmd, buffer, size - 1, I2C_MASTER_ACK );
    
    i2c_master_read_byte( cmd, buffer + size - 1, I2C_MASTER_NACK );

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin( i2c_num, cmd, 1000 / portTICK_RATE_MS );

    i2c_cmd_link_delete( cmd );

    return ret;
}