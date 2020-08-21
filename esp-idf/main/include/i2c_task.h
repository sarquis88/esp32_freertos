#include "main_task.h"

#define I2C_TASK_PRIORITY			( tskIDLE_PRIORITY + 2              )
#define I2C_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 2024   )
#define I2C_LOGGING_TAG             ( ( const char * ) "I2C"            )

#define I2C_MASTER_FREQ_HZ          ( ( uint32_t ) 500000   )
#define ESP_SLAVE_ADDR              ( ( uint8_t ) 104       )
#define READ_BIT                    ( 1                     )

void prvI2CTask                 ( void *                        );
void start_i2c_task             ( void                          );
esp_err_t i2c_master_init       ( void                          );
esp_err_t i2c_master_read_slave ( i2c_port_t, uint8_t *, size_t );
