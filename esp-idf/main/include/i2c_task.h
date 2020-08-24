#include "main_task.h"

#define I2C_DELAY_MS				( ( uint16_t ) 2000                 )
#define I2C_TASK_PRIORITY			( tskIDLE_PRIORITY + 2              )
#define I2C_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 3072   ) // *4 -> bytes
#define I2C_LOGGING_TAG             ( ( const char * ) "I2C"            )
#define I2C_TASK_WATERMARK          ( 1                                 )

#define I2C_MASTER_FREQ_HZ          ( ( uint32_t ) 500000   )
#define I2C_SLAVE_ADDR              ( ( uint8_t ) 104       ) // 0x28
#define READ_BIT                    ( 1                     )

#define I2C_MASTER_BUFFER_SIZE      ( 1024                  ) * 100
#define I2C_SLAVE_BUFFER_SIZE       ( 1024                  )

void prvI2CTask                 ( void *                        );
void start_i2c_task             ( void                          );
esp_err_t i2c_master_init       ( void                          );
esp_err_t i2c_slave_init        ( void                          );
esp_err_t i2c_master_read_slave ( i2c_port_t, uint8_t *, size_t );