/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Must be included if using STM32F4 Discovery board or processor
#include "stm32f4xx.h"

// Must be included to setup general purpose I/O
// Some peripherals require the setup of RCC (Reset and clock controller)
#include "stm32f4xx_rcc.h"

// GPIO initialization and other minor stuff
#include "init_board.h"

// UART driver
#include "uart_driver.h"

void blikej(void *p) {

	while (1) {

		GPIO_ToggleBits(GPIOC, GPIO_Pin_2);
		vTaskDelay(1000);
	}
}

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

    init_USART4(9600); // initialize USART4 @ 9600 baud
    USART_puts(UART4, "Init complete! Hello World!rn"); // just send a message to indicate that it works
    gpio_init();

	xTaskCreate(blikej, (char*) "blikej", 1024, NULL, 2, NULL);
	xTaskCreate(pocitej, (char*) "pocitej", 1024, NULL, 2, NULL);

	vTaskStartScheduler();

	return 0;
}
