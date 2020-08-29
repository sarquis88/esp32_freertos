#include "../include/main_task.h"

/* ######################################################################## */


#define BLINKER_TASK_ON				0
#define INTRECEIVER_TASK_ON			0
#define WIFICONNECTION_TASK_ON		0
#define WIFISCANNER_TASK_ON			0
#define TASKSLIST_TASK_ON			0

/* ######################################################################## */

void 
app_main( void )
{	
	/* Starts accelerometer senses */
	start_accelerometer_task();

	#if TASKSLIST_TASK_ON == 1
		/* Starts tasks listing */
		start_taskslist_task();
	#endif
}
