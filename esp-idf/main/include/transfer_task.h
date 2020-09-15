#include "main_task.h"

using namespace std;

#define TRANSFER_DELAY_MS				    ( ( uint16_t ) 2000                 )
#define TRANSFER_LONG_DELAY_MS		        ( ( uint16_t ) 10000                )
#define TRANSFER_TASK_PRIORITY			    ( tskIDLE_PRIORITY + 2              )
#define TRANSFER_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 2024   )
#define TRANSFER_LOGGING_TAG                ( ( const char * ) "Transfer" )

void start_transfer_task            ( xQueueHandle*, xQueueHandle*	);
void prvTransferTask	            ( void *                        );
esp_err_t event_handler             ( void *, system_event_t *      );
void socket_config                  ( void                          );
void send_message                   ( string                        );
void wifi_config                    ( string, string                );