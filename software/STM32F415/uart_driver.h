/*
 * init_board.h
 *
 * Created: 24.8.2014 21:05:19
 *  Author: klaxalk
 */

#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

#include "system.h"

#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src

// Must be included if using GPIO (General purpose I/O) peripheral
#include "stm32f4xx_gpio.h"

// Must be included to setup general purpose I/O
// Some peripherals require the setup of RCC (Reset and clock controller)
#include "stm32f4xx_rcc.h"

// I recommend you have a look at these in the ST firmware folder
#include <misc.h>

#define MAX_STRLEN 12 // this is the maximum string length of our string in characters
volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string

void init_USART4(uint32_t baudrate);

portBASE_TYPE usart4PutChar(char ch);
void usart4PutString(volatile char *s);

#endif /* INIT_BOARD_H_ */
