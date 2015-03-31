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

// Must be included if using STM32F4 Discovery board or processor
#include "stm32f4xx.h"

// Must be included to setup general purpose I/O
// Some peripherals require the setup of RCC (Reset and clock controller)
#include "stm32f4xx_rcc.h"

// Must be included if using GPIO (General purpose I/O) peripheral
#include "stm32f4xx_gpio.h"

// UART driver
#include "uart_driver.h"

#include "kalman/elevator/elevatorKalman.h"
#include "kalman/aileron/aileronKalman.h"

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

// queues for usart
QueueHandle_t * usartRxQueue;
QueueHandle_t * usartTxQueue;

/* -------------------------------------------------------------------- */
/*	Messages types for communication between tasks						*/
/* -------------------------------------------------------------------- */

enum comm2mpcMessageType_t {SETPOINT, TRAJECTORY};

// setpoint message
typedef struct {

	enum comm2mpcMessageType_t messageType;

	float elevatorReference[5];
	float aileronReference[5];

} comm2mpcMessage_t;

// px4flow fresh data message
typedef struct {

	float elevatorSpeed;
	float aileronSpeed;
	float elevatorInput;
	float aileronInput;
} comm2kalmanMessage_t;

// mpc output message
typedef struct {

	float elevatorOutput;
	float aileronOutput;
	float elevatorSetpoint;
	float aileronSetpoint;
} mpc2commMessage_t;

// message to reset the kalman states
typedef struct {

	float elevatorPosition;
	float aileronPosition;
} resetKalmanMessage_t;

// kalman output message (to mpc)
typedef struct {

	float elevatorData[NUMBER_OF_STATES_ELEVATOR];
	float aileronData[NUMBER_OF_STATES_AILERON];
} kalman2mpcMessage_t;

// kalman output message (to comm)
typedef struct {

	float elevatorData[NUMBER_OF_STATES_ELEVATOR];
	float aileronData[NUMBER_OF_STATES_AILERON];
} kalman2commMessage_t;

/* -------------------------------------------------------------------- */
/*	Queues for communication between tasks								*/
/* -------------------------------------------------------------------- */

// queue from commTask to kalmanTask
QueueHandle_t * comm2kalmanQueue;

// queue from mpcTask to commTask
QueueHandle_t * mpc2commQueue;

// queue from kalman to mpc
QueueHandle_t * kalman2mpcQueue;

// queue from kalmanTask to commTask
QueueHandle_t * kalman2commQueue;

// queue from commTask to mpcTask
QueueHandle_t * comm2mpcQueue;

// queue to reset a kalman
QueueHandle_t * resetKalmanQueue;

// queue to set kalman's position to a particular value
QueueHandle_t * setKalmanQueue;

#define led_toggle() GPIO_ToggleBits(GPIOC, GPIO_Pin_2)
#define led_on() GPIO_WriteBit(GPIOC, GPIO_Pin_2, 1)
#define led_off() GPIO_WriteBit(GPIOC, GPIO_Pin_2, 0)

// Initialize the board
void boardInit();

// Initialization of GPIO ports
void gpioInit();

#endif /* SYSTEM_H_ */
