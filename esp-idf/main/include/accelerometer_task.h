#include "main_task.h"

#define ACCELEROMETER_TASK_DELAY_MS				( ( uint16_t ) 1000                 )
#define ACCELEROMETER_TASK_PRIORITY			    ( tskIDLE_PRIORITY + 2              )
#define ACCELEROMETER_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 6000   ) 
#define ACCELEROMETER_TASK_TAG                  ( ( const char* ) "Accelerometer"   )
#define ACCELEROMETER_TASK_WATERMARK            ( 0                                 )

#define I2C_MASTER_FREQ_HZ          ( ( uint32_t ) 1000   )
#define I2C_SLAVE_ADDR              ( 0x68                  )

#define NVS_STORAGE_NAME            ( ( const char* ) "storage"     )
#define NVS_DATA_KEY_NAME           ( ( const char* ) "data"        )
#define NVS_INDEX_KEY_NAME          ( ( const char* ) "index"       )
#define NVS_MAX_INDEX_VALUE         ( ( uint8_t ) 50                )

#define MPU_AXIS_COUNT              ( ( uint8_t ) 3                     )
#define MPU_FIFO_SIZE               ( ( size_t ) 1024                   )
#define MPU_GROUP_SIZE              ( ( size_t ) 150                    )

#define RTC_MPU_DATA_SIZE           ( ( uint8_t ) 10000                  )

void prvAccelerometerTask       ( void *                                );
void start_accelerometer_task   ( void                                  );

esp_err_t nvs_init              ( void                          );
esp_err_t nvs_read              ( uint16_t*, size_t*, uint8_t   );
esp_err_t nvs_write             ( uint16_t*, size_t, uint8_t    );
esp_err_t nvs_check             ( void                          );
esp_err_t nvs_get_entry_count   ( size_t*                       );
esp_err_t nvs_get_index         ( uint8_t*                      );
esp_err_t nvs_set_index         ( uint8_t                       );

esp_err_t i2c_init              ( void              );

void mpu_init                   ( void              );
void mpu_check_reg_values       ( void              );

void start_deep_sleep_mode      ( void              );
void check_rtc_values           ( void              );

