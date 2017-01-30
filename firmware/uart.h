#ifndef _UART_H
#define _UART_H

#include <avr/io.h>

/* get next character from uart (receive) */
extern unsigned char uart_getc (void);
/* put next character to uart (send) */
extern void uart_putc (unsigned char c);

#endif
