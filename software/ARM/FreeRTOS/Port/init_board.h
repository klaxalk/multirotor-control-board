/*
 * init_board.h
 *
 * Created: 24.8.2014 21:05:19
 *  Author: klaxalk
 */

#ifndef INIT_BOARD_H_
#define INIT_BOARD_H_

// Must be included to setup general purpose I/O
// Some peripherals require the setup of RCC (Reset and clock controller)
#include "stm32f4xx_rcc.h"

// Must be included if using GPIO (General purpose I/O) peripheral
#include "stm32f4xx_gpio.h"

/**********************************************************************************
 *
 * The GPIO_InitTypeDef is a structure defined in the stm32f4xx_gpio.h file.
 * This is a simple way to use ST's library to configure the I/O.
 *
 * This is a common setup for the other peripherals as well.  Each peripheral will have
 * different properties which can be found in the header files and source files.
 *
**********************************************************************************/

GPIO_InitTypeDef  GPIO_InitStruct;


// Initialization of GPIO ports
void gpio_init();

#endif /* INIT_BOARD_H_ */
