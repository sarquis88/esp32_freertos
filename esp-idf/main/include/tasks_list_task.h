#include "main_task.h"

#define TASKSLIST_DELAY_MS				    ( ( uint16_t ) 5000                 )
#define TASKSLIST_TASK_PRIORITY			    ( tskIDLE_PRIORITY + 2              )
#define TASKSLIST_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 2048   )
#define TASKSLIST_LOGGING_TAG               ( ( const char * ) "TasksList"      )

void start_taskslist_task           ( void                      );
void prvTasksListTask	            ( void *                    );