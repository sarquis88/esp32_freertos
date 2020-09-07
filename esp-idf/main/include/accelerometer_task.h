#include "main_task.h"

#define ACCELEROMETER_TASK_DELAY_MS				( ( uint16_t ) 250                  )
#define ACCELEROMETER_TASK_PRIORITY			    ( tskIDLE_PRIORITY + 2              )
#define ACCELEROMETER_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 6000   ) 
#define ACCELEROMETER_TASK_TAG                  ( ( const char* ) "Accelerometer"   )
#define ACCELEROMETER_TASK_WATERMARK            ( 0                                 )
#define ACCELEROMETER_TASK_LOGGING              ( 1                                 )

#define I2C_MASTER_FREQ_HZ          ( ( uint32_t ) 500000   )
#define I2C_SLAVE_ADDR              ( 0x68                  )

#define NVS_STORAGE_NAME            ( ( const char* ) "storage"     )
#define NVS_DATA_KEY_NAME           ( ( const char* ) "data"        )
#define NVS_INDEX_KEY_NAME          ( ( const char* ) "index"       )
#define NVS_MAX_INDEX_VALUE         ( ( uint8_t ) 4                 )

#define MPU_AXIS_COUNT              ( ( uint8_t ) 3                     )
#define MPU_FIFO_SIZE               ( ( size_t ) 1024                   )
#define MPU_GROUP_SIZE              ( ( size_t ) 150                    )

void prvAccelerometerTask       ( void *                                );
void start_accelerometer_task   ( void                                  );

esp_err_t i2c_init              ( void              );

void mpu_init                   ( void              );
void mpu_check_reg_values       ( void              );

void IRAM_ATTR gpio_int_handler ( void*             );

