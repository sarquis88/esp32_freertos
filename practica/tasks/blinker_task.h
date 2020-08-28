#include "main_task.h"

#define BLINKER_DELAY_MS				( ( uint16_t ) 100                  )
#define BLINKER_TASK_PRIORITY			( tskIDLE_PRIORITY + 2              )
#define BLINKER_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 2024   )

void start_blinker_task         ( void      );
void prvBlinkerTask             ( void *    );