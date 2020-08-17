#include "main_task.h"

#define WIFISCANNER_DELAY_MS				( ( uint16_t ) 5000                 )
#define WIFISCANNER_TASK_PRIORITY			( tskIDLE_PRIORITY + 2              )
#define WIFISCANNER_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 2024   )
#define WIFISCANNER_LOGGING_TAG             ( ( const char * ) "WifiScanner"    )

#define MAX_APS_LIST_SIZE					( 3                                 )

void start_wifiscanner_task     ( void      );
void prvWifiScannerTask	        ( void*     );
void scan_wifi                  ( string*   );