#include "main_task.h"

#define ACCELEROMETER_TASK_DELAY_MS		( ( uint16_t ) 1000                 )
#define ACCELEROMETER_TASK_PRIORITY		( tskIDLE_PRIORITY + 2              )
#define ACCELEROMETER_TASK_STACK_SIZE	( configMINIMAL_STACK_SIZE + 8096   ) 
#define ACCELEROMETER_TASK_TAG          ( ( const char* ) "Accelerometer"   )

#define I2C_MASTER_FREQ_HZ              ( ( uint32_t ) 500000               )
#define I2C_SLAVE_ADDR                  ( 0x68                              )

#define MPU_AXIS_COUNT                  ( ( uint8_t     ) 3                 )
#define MPU_FIFO_SIZE                   ( ( size_t      ) 1024              )
#define MPU_GROUP_SIZE                  ( ( size_t      ) 170               )
#define MPU_AX_OFFSET                   ( ( int16_t ) -3200                 )
#define MPU_AY_OFFSET                   ( ( int16_t ) -1480                 )
#define MPU_AZ_OFFSET                   ( ( int16_t ) -350                  )

/* 
    Max = ~7900
*/
#define RAM_DATA_SIZE                   ( ( size_t ) 500                   )  
/* 
    Max = ~2400000
*/
#define FILESYSTEM_DATA_SIZE            ( ( size_t ) 500                  )  

/*
    Function called by the main task
    It prepares and starts the accelerometer task
    The task should always be started with this function
    @param accelerometer and transfer tasks queues, respectively
*/
void start_accelerometer_task           ( xQueueHandle*, xQueueHandle*      );

/*
    Accelerometer task
    Manages the MPU sensor and the sleep modes
    When storage is full, it comunicates with the transfer task for transmiting
    the data
*/
void prvAccelerometerTask               ( void *                            );

/*
    I2C module configuration and initialization
    @return ESP error type
*/
esp_err_t i2c_init                      ( void                              );

/*
    MPU sensor configuration and initialization
*/
void mpu_init                           ( void                              );

/*
    Send the micro into deep sleep mode
    It can only be awakened by the pin 2
*/
void start_deep_sleep_mode              ( void                              );

uint8_t get_scaled_module               ( int16_t [ MPU_AXIS_COUNT ]        );

/*
    Send messages to the transfer task for start the data transmition
*/
void send_data_and_wait                 ( uint32_t                          );

esp_err_t write_to_filesystem           ( uint8_t*, size_t                  );



