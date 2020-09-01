#include "main_task.h"

#define ACCELEROMETER_TASK_DELAY_MS				( ( uint16_t ) 1000                 )
#define ACCELEROMETER_TASK_PRIORITY			    ( tskIDLE_PRIORITY + 2              )
#define ACCELEROMETER_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 3072   ) 
#define ACCELEROMETER_LOGGING_TAG               ( ( const char* ) "Accelerometer"   )
#define ACCELEROMETER_TASK_WATERMARK            ( 0                                 )
#define ACCELEROMETER_TASK_LOGGING              ( 1                                 )

#define I2C_MASTER_FREQ_HZ          ( ( uint32_t ) 500000   )
#define I2C_SLAVE_ADDR              ( 0x68                  )

#define NVS_MAX_INDEX               ( 629                               )
#define NVS_STORAGE_NAME            ( ( const char* ) "mpudata"         )
#define NVS_INDEX_KEY               ( ( const char* ) "index"           )

#define MPU_AXIS_COUNT              ( ( uint8_t ) 3                     )
#define MPU_FIFO_SIZE               ( ( size_t ) 1024                   )

void prvAccelerometerTask       ( void *                        );
void start_accelerometer_task   ( void                          );

esp_err_t nvs_init              ( void                                  );
esp_err_t nvs_read              ( const char*, const char*, uint64_t*   );
esp_err_t nvs_write             ( const char*, const char*, uint64_t    );
esp_err_t nvs_check_values      ( void                                  );

esp_err_t i2c_init              ( void                          );

void mpu_init                   ( void                          );
void mpu_check_reg_values  ( void                          );

void start_deep_sleep_mode      ( void                                      );
void log_results                ( uint16_t, int16_t*, uint64_t, uint16_t    );

