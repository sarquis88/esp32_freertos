#include "main_task.h"

#define WIFICONNECTION_DELAY_MS				    ( ( uint16_t ) 2000                 )
#define WIFICONNECTION_LONG_DELAY_MS		    ( ( uint16_t ) 10000                )
#define WIFICONNECTION_TASK_PRIORITY			( tskIDLE_PRIORITY + 2              )
#define WIFICONNECTION_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 2024   )
#define WIFICONNECTION_LOGGING_TAG              ( ( const char * ) "WifiConnection" )

void start_wificonnection_task      ( void                      );
void prvWifiConnectionTask	        ( void *                    );
esp_err_t event_handler             ( void *, system_event_t *  );