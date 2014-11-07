/*
 * File name: debugcomm.c
 * Date:      2014/11/07 08:41
 * Author:    Jan Chudoba
 */

#include "debugcomm.h"
#include "system.h"

#ifdef WITH_PRINTF
#include <stdarg.h>
#endif

void debugMessage(char * message)
{
	usartBufferPutString(DEBUG_USART_BUFFER, message, 0);
}

#ifdef WITH_PRINTF
void debugMessageF(const char * message, ...)
{
	char buffer[64];
	va_list arg;
	va_start(arg, message);
	vsnprintf(buffer, 64, message, arg);
	va_end(arg);
	debugMessage((char*) buffer);
}

#endif

/* end of debugcomm.c */
