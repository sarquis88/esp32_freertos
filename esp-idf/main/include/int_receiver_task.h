#include "main_task.h"

#define mainINTRECEIVER_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define configINTRECEIVER_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE + 2024 )

void start_intreceiver_task      ( void      );
void IRAM_ATTR gpio_int_handler  ( void *    );
void prvIntReceiverTask          ( void *    );