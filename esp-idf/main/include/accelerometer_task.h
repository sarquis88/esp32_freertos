#include "main_task.h"

#define ACCELEROMETER_TASK_DELAY_MS		( ( uint16_t ) 1000                 )
#define ACCELEROMETER_TASK_PRIORITY		( tskIDLE_PRIORITY + 2              )
#define ACCELEROMETER_TASK_STACK_SIZE	( configMINIMAL_STACK_SIZE + 6000   ) 
#define ACCELEROMETER_TASK_TAG          ( ( const char* ) "Accelerometer"   )

#define I2C_MASTER_FREQ_HZ              ( ( uint32_t ) 1000                 )
#define I2C_SLAVE_ADDR                  ( 0x68                              )

#define MPU_AXIS_COUNT                  ( ( uint8_t     ) 3                 )
#define MPU_FIFO_SIZE                   ( ( size_t      ) 1024              )
#define MPU_GROUP_SIZE                  ( ( size_t      ) 170               )
#define MPU_AX_OFFSET                   ( ( int16_t     ) -3700             )
#define MPU_AY_OFFSET                   ( ( int16_t     ) 1500              )
#define MPU_AZ_OFFSET                   ( ( int16_t     ) -270              )

#define RTC_MPU_DATA_SIZE               ( ( uint16_t    ) 100               )   //7990

void prvAccelerometerTask               ( void *                            );
void start_accelerometer_task           ( xQueueHandle*, xQueueHandle*      );

esp_err_t i2c_init                      ( void                              );

void mpu_init                           ( void                              );

void start_deep_sleep_mode              ( void                              );
void clear_rtc_storage                  ( void                              );
void send_data_and_wait                 ( void                              );

