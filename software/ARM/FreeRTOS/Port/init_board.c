/*
 * init_board.h
 *
 * Created: 24.8.2014 21:05:19
 *  Author: klaxalk
 */

#include "init_board.h"

void gpio_init() {

		/**********************************************************************************
	     *
	     * This enables the peripheral clock to the GPIOD module.  This is stated in
	     * the beginning of the stm32f4xx.gpio.c source file.
	     *
	    **********************************************************************************/
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	    /**********************************************************************************
	     *
	     * This block of code defines the properties of the GPIO port.
	     * The different options of each item can be found in the stm32f4xx_gpio.h header file
	     * Every GPIO configuration should have the following initialized
	     *
	     * GPIO_InitStruct.GPIO_Pin
	     * GPIO_InitStruct.GPIO_Mode
	     * GPIO_InitStruct.GPIO_Speed
	     * GPIO_InitStruct.GPIO_OType
	     * GPIO_InitStruct.GPIO_PuPd
	     * GPIO_Init(GPIOx, &GPIO_InitStruct); (x represents port - A, B, C, D, E, F, G, H, I)
	     *
	    **********************************************************************************/

	    //GPIO_InitStruct.GPIO_Pin configures the pins that will be used.
	    //In this case we will use the LED's off of the discovery board which are on
	    //PortD pins 12, 13, 14 and 15
	    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;

	    //PIO_InitStruct.GPIO_Mode configures the pin mode the options are as follows
	    // GPIO_Mode_IN (Input Mode)
	    // GPIO_Mode_OUT (Output Mode)
	    // GPIO_Mode_AF (Alternate Function)
	    // GPIO_Mode_AN (Analog Mode)
	    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;

	    //GPIO_InitStruct.GPIO_Speed configures the clock speed, options are as follows
	    // GPIO_Speed_2MHz
	    // GPIO_Speed_25MHz
	    // GPIO_Speed_50MHz
	    // GPIO_Speed_100MHz
	    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	    //GPIO_InitStruct.GPIO_OType configures the pin type, options are as follows
	    // GPIO_OType_PP (Push/Pull)
	    // GPIO_OType_OD (Open Drain)
	    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;

	    //Configures pullup / pulldown resistors on pin, options are as follows
	    // GPIO_PuPd_NOPULL (Disables internal pullup and pulldown resistors)
	    // GPIO_PuPd_UP (Enables internal pullup resistors)
	    // GPIO_PuPd_DOWN (Enables internal pulldown resistors)
	    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	    //This finally passes all the values to the GPIO_Init function
	    //which takes care of setting the corresponding bits.
	    GPIO_Init(GPIOC, &GPIO_InitStruct);

	    /**********************************************************************************
	     *
	     * This enables the peripheral clock to the GPIOA module.  This is stated in
	     * the beginning of the stm32f4xx.gpio.c source file.
	     *
	    **********************************************************************************/
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	    /**********************************************************************************
	     *
	     * This block of code defines the properties of the GPIOA port.
	     * We are defining Pin 0 as a digital input with a pulldown resistor
	     * to detect a high level.  Pin 0 is connected to the 3.3V source
	     *
	   **********************************************************************************/
	    /*
	    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	    GPIO_Init(GPIOA, &GPIO_InitStruct);
	    */
}
