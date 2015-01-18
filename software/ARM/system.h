/*
 * system.h
 *
 * Created: 24.8.2014 21:05:19
 *  Author: klaxalk
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "CMatrixLib.h"

// Must be included if using STM32F4 Discovery board or processor
#include "stm32f4xx.h"

// Must be included to setup general purpose I/O
// Some peripherals require the setup of RCC (Reset and clock controller)
#include "stm32f4xx_rcc.h"

// Must be included if using GPIO (General purpose I/O) peripheral
#include "stm32f4xx_gpio.h"

// UART driver
#include "uart_driver.h"

/**********************************************************************************
 *
 * The GPIO_InitTypeDef is a structure defined in the stm32f4xx_gpio.h file.
 * This is a simple way to use ST's library to configure the I/O.
 *
 * This is a common setup for the other peripherals as well.  Each peripheral will have
 * different properties which can be found in the header files and source files.
 *
**********************************************************************************/

GPIO_InitTypeDef	GPIO_InitStruct;

// uart input queue
extern QueueHandle_t * uartQueue;

// queue from commTask to kalmanTask
extern QueueHandle_t * comm2kalmanQueue;

#define led_toggle() GPIO_ToggleBits(GPIOC, GPIO_Pin_2)
#define led_on() GPIO_WriteBit(GPIOC, GPIO_Pin_2, 1)
#define led_off() GPIO_WriteBit(GPIOC, GPIO_Pin_2, 0)

// Initialize the board
void boardInit();

// Initialization of GPIO ports
void gpioInit();

#endif /* SYSTEM_H_ */
