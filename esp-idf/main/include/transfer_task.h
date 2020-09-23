#include "main_task.h"

#define TRANSFER_TASK_DELAY         ( 1000                                          )
#define TRANSFER_TASK_PRIORITY		( tskIDLE_PRIORITY + 2                          )
#define TRANSFER_TASK_STACK_SIZE	( configMINIMAL_STACK_SIZE + 8096               )
#define TRANSFER_TASK_TAG           ( ( const char * ) "Transfer"                   )

#define HTTP_URL                    ( ( const char * ) "http://192.168.100.3:8080"  )
#define HTTP_KEY                    ( ( const char * ) "Accel"                      )
#define HTTP_CHUNK_SIZE             ( 100                                           ) // has to be multiple of RTC_MPU_DATA_SIZE

/*
    Function called by the main task.
    It prepares and starts the transfer task.
    The task should always be started with this function.
    @param transfer and accelerometer tasks queues, respectively
*/
void start_transfer_task            ( xQueueHandle*, xQueueHandle*	                );

/*
    Transfer task function
    In charge of receiving the data from the accelerometer task and 
    transmitting it through WiFi
*/
void prvTransferTask	            ( void *                                        );

/*
    Event handler for WiFi
    @param information provided by the events
    @return ESP error type
*/
esp_err_t event_handler             ( void *, system_event_t *                      );

/*
    WiFi configuration
    @param SSID and password, respectively
*/
void wifi_config                    ( string, string                                );

/*
    Send data through HTTP in JSON format
    @param JSON key and value, respectively
*/
void http_send_json                 ( string, string                                );

/*
    Send data through HTTP in plain text format
    @param data pointer and length, respectively
*/
void http_send_plain                ( uint8_t*, uint16_t                            );