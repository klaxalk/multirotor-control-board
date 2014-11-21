// GPIO initialization and other minor stuff
#include "system.h"
#include "commTask.h"

void pocitej(void *p) {

    float baf;
    long lek;
    long max = 7000000;

    while(1) {

        USART_puts(UART4, "done\n\r"); // just send a message to indicate that it works

    	for (lek = 0; lek < max; ++lek) {

    		baf = baf * ((float) 1.02);
		}
    }
}


int main(void)
{

	// initialize the hardware
    boardInit();

	/* -------------------------------------------------------------------- */
	/*	Start the communication task routine								*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(commTask, (char*) "commTask", 1024, NULL, 2, NULL);

	xTaskCreate(pocitej, (char*) "pocitej", 1024, NULL, 2, NULL);

	/* -------------------------------------------------------------------- */
	/*	Start the FreeRTOS scheduler										*/
	/* -------------------------------------------------------------------- */
	vTaskStartScheduler();

	return 0;
}
