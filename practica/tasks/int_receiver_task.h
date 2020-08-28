#include "main_task.h"

#define INTRECEIVER_TASK_PRIORITY			( tskIDLE_PRIORITY + 2              )
#define INTRECEIVER_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE + 2024   )
#define INTRECEIVER_LOGGING_TAG             ( ( const char * ) "IntReceiver"    )

void start_intreceiver_task      ( void      );
void IRAM_ATTR gpio_int_handler  ( void *    );
void prvIntReceiverTask          ( void *    );