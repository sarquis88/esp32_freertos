#include "main_task.h"

#define TRANSFER_TASK_DELAY         ( 1000                                          )
#define TRANSFER_TASK_PRIORITY		( tskIDLE_PRIORITY + 2                          )
#define TRANSFER_TASK_STACK_SIZE	( configMINIMAL_STACK_SIZE + 8096               )
#define TRANSFER_TASK_TAG           ( ( const char * ) "Transfer"                   )

#define HTTP_URL                    ( ( const char * ) "http://192.168.100.3:8080"  )
#define HTTP_KEY                    ( ( const char * ) "Accel"                      )
#define HTTP_CHUNK_SIZE             ( 100                                           ) // has to be multiple of RTC_MPU_DATA_SIZE

void start_transfer_task            ( xQueueHandle*, xQueueHandle*	                );
void prvTransferTask	            ( void *                                        );
esp_err_t event_handler             ( void *, system_event_t *                      );
void socket_config                  ( void                                          );
void send_message                   ( uint8_t                                       );
void wifi_config                    ( string, string                                );
void http_send_json                 ( string, string                                );
void http_send_plain                ( uint8_t*, uint16_t                            );