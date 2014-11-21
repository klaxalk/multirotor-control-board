/*
 * commTask.c
 *
 *  Author: Tomas Baca
 */

#include "commTask.h"

void commTask(void *p) {

	char inChar;

	while (1) {

		if (xQueueReceive(uartQueue, &inChar, 1)) {

			led_toggle();
			USART_SendData(UART4, inChar);
		}
	}
}
