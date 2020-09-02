#include "../include/main_task.h"

/* ######################################################################## */

#define ACCELEROMETER_TASK_ON		1
#define TASKSLIST_TASK_ON			0

/* ######################################################################## */

void 
app_main( void )
{	
	#if ACCELEROMETER_TASK_ON == 1
	/* Starts accelerometer senses */
	start_accelerometer_task();
	#endif

	#if TASKSLIST_TASK_ON == 1
	/* Starts tasks listing */
	start_taskslist_task();
	#endif
}
