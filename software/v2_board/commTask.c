/*
 * communicationTask.c
 *
 * Created: 11.9.2014 11:17:03
 *  Author: Tomas Baca
 */ 

#include "commTask.h"
#include "system.h"
#include "ioport.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart_driver_RTOS.h"

extern volatile uint16_t RCchannel[9];
extern UsartBuffer * usart_buffer_xbee;

void commTask(void *p) {
	
	unsigned char inChar;
	
	while (1) {

		// xbee received
		if (usartBufferGetByte(usart_buffer_xbee, &inChar, 0)) {
					
			if (inChar == 'x') {
				
				int i;
				for (i = 0; i < 4; i++) {
										
					usartBufferPutInt(usart_buffer_xbee, RCchannel[i], 10, 10);
					usartBufferPutString(usart_buffer_xbee, ", ", 10);
				}
				usartBufferPutString(usart_buffer_xbee, "\r\n", 10);
			}
		}
		
		taskYIELD();
	}
}