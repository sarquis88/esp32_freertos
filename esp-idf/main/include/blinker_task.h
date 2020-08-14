#include "main_task.h"

#define DELAY_BLINKER_MS					( ( uint16_t ) 100 )

#define mainBLINKER_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define configBLINKER_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE + 2024 )

void start_blinker_task          ( void      );
void prvBlinkerTask          ( void *    );