/*
 * File name: debugcomm.h
 * Date:      2014/11/07 08:41
 * Author:    Jan Chudoba
 */

#ifndef __DEBUGCOMM_H__
#define __DEBUGCOMM_H__

#ifdef WITH_PRINTF
#include <stdio.h>
#endif

#ifndef DEBUG_USART_BUFFER
#define DEBUG_USART_BUFFER usart_buffer_stm
// TODO: [low] check if STM communication is disabled
#endif

void debugMessage(char * message);

#ifdef WITH_PRINTF
void debugMessageF(const char * message, ...);
#endif

#endif

/* end of debugcomm.h */
