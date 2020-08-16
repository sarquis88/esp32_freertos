#include "main_task.h"

#define WIFICONNECTION_DELAY_MS				    ( ( uint16_t ) 2000                 )
#define WIFICONNECTION_TASK_PRIORITY			( tskIDLE_PRIORITY + 2              )
#define WIFICONNECTION_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 2024   )

void start_wificonnection_task      ( void                      );
void connect_to_ap                  ( uint8_t* , uint8_t*       );
void prvWifiConnectionTask	        ( void *                    );
esp_err_t event_handler             ( void *, system_event_t *  );