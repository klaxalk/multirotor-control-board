/*
 * init_board.h
 *
 * Created: 24.8.2014 21:05:19
 *  Author: klaxalk
 */

#include "uart_driver.h"
#include "system.h"

/* This funcion initializes the USART4 peripheral
 *
 * Arguments: baudrate --> the baudrate at which the USART is
 * 						   supposed to operate
 */
void init_USART4(uint32_t baudrate) {

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART4 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART4
	 * note that only USART4 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART4, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(UART4, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	USART_ITConfig(UART4, USART_IT_TXE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff


	// finally this enables the complete USART1 peripheral
	USART_Cmd(UART4, ENABLE);
}

/* This function is used to transmit a string of characters via
 * the USART specified in USARTx.
 *
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 *
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 *
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */

portBASE_TYPE usart4PutChar(char ch) {

	if(xQueueSend(usartTxQueue, &ch, 10) == pdPASS) {

		USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
		return pdTRUE;
    } else {
    	return pdFAIL;
    }
}

void usart4PutString(volatile char *s) {

	while (*s) {

		usart4PutChar(*s);
		//while( !(UART4->SR & 0x00000040) );
		//		USART_SendData(UART4, *s);
		*s++;
	}
}

// this is the interrupt request handler (IRQ) for ALL USART4 interrupts
void UART4_IRQHandler(void) {

	long xHigherPriorityTaskWoken = pdFALSE;
	uint8_t ch;

	// check if the USART4 receive interrupt flag was set
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) {

		ch = (uint8_t) USART_ReceiveData(UART4);

		xQueueSendToBackFromISR(usartRxQueue, &ch, &xHigherPriorityTaskWoken);
	}

	if (USART_GetITStatus(UART4, USART_IT_TXE) != RESET) {

		if(xQueueReceiveFromISR(usartTxQueue, &ch, &xHigherPriorityTaskWoken)) {

			USART_SendData(UART4, ch);

		} else {

			//disable Transmit Data Register empty interrupt
			USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
		}
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

