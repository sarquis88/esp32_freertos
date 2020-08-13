#include "../include/common.h"

#define mainINTRECEIVER_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define configINTRECEIVER_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE + 2024 )

#define GPIO_PIN_19   					( ( gpio_num_t ) 19 )
#define GPIO_INPUT_PIN_SEL  			( 1ULL<<GPIO_PIN_19 )

void start_intreceiver_task      ( void      );
void IRAM_ATTR gpio_int_handler  ( void *    );
void prvIntReceiverTask          ( void *    );