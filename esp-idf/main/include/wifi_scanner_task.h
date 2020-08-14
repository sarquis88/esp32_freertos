#include "main_task.h"

#define DELAY_WIFISCANNER_MS				( ( uint16_t ) 5000                 )
#define WIFISCANNER_TASK_PRIORITY			( tskIDLE_PRIORITY + 2              )
#define WIFISCANNER_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 2024   )
#define MAX_APS_LIST_SIZE					( 3                                 )

void start_wifiscanner_task     ( void      );
void prvWifiScannerTask	        ( void*     );
void scan_wifi                  ( string*   );