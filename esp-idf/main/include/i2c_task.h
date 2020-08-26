#include "main_task.h"

#define I2C_DELAY_MS				( ( uint16_t ) 500                  )
#define I2C_TASK_PRIORITY			( tskIDLE_PRIORITY + 2              )
#define I2C_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 3072   ) // *4 -> bytes
#define I2C_LOGGING_TAG             ( ( const char* ) "I2C"             )
#define I2C_TASK_WATERMARK          ( 0                                 )

#define I2C_MASTER_FREQ_HZ          ( ( uint32_t ) 500000   )
#define I2C_SLAVE_ADDR              ( ( uint8_t ) 104       ) // 0x28
#define I2C_MASTER_READ             ( 1                     )
#define I2C_MASTER_WRITE            ( 0                     )

#define NVS_MAX_INDEX               ( 629                               )
#define NVS_STORAGE_NAME            ( ( const char* ) "mpudata"         )

#define REG_AX_L                0x3C
#define REG_AX_H                0x3B
#define REG_AY_L                0x3E
#define REG_AY_H                0x3D
#define REG_AZ_L                0x40
#define REG_AZ_H                0x3F

void prvI2CTask                 ( void *                        );
void start_i2c_task             ( void                          );

esp_err_t nvs_read              ( string, string, uint64_t*     );
esp_err_t nvs_write             ( string, string, uint64_t      );

esp_err_t i2c_init              ( void                          );
esp_err_t i2c_receive_byte      ( uint8_t *                     );
esp_err_t i2c_ask_mpu_register  ( uint8_t                       );
