#include "../include/tasks_list_task.h"

void 
start_taskslist_task()
{
    /* Task creation */
	xTaskCreate( 	prvTasksListTask, "tasksList", 
					TASKSLIST_TASK_STACK_SIZE, NULL, 
					TASKSLIST_TASK_PRIORITY, NULL );
}

void prvTasksListTask( void *pvParameters )
{

    for( ; ; )
    {
        BaseType_t number_of_tasks = uxTaskGetNumberOfTasks();
        char buffer[ BUFFER_SIZE * number_of_tasks ];

        vTaskDelay( TASKSLIST_TASK_DELAY_MS / portTICK_PERIOD_MS);

        vTaskList( buffer );

        ESP_LOGI( TASKSLIST_LOGGING_TAG, "\nNumber of tasks:\t%d\n\nName\t\tState\tPrior.\tStack\tNum\n%s", number_of_tasks, buffer );
    }

    vTaskDelete( NULL );
}