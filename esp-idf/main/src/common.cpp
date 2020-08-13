#include "../include/common.h"

static SemaphoreHandle_t cout_mutex = NULL;

void
safe_cout( string msg )
{
	if( cout_mutex == NULL )
	{
		cout_mutex = xSemaphoreCreateBinary();
		xSemaphoreGive( cout_mutex );
	}

	xSemaphoreTake( cout_mutex, portMAX_DELAY );

	cout << msg << endl;

	xSemaphoreGive( cout_mutex );
}