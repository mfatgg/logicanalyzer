;**** Fast Softuart implementation ************************
;*
;* - based on AVR305 application note from Atmel
;* - slightly changed for Atmega circuits
;* - multiple baudrate examples given in comments
;* - convert to c-callable function, remove test main loop
;**********************************************************

#include <avr/io.h>

;***** Pin definitions

RxD		=	0			;Receive pin is PD0
TxD		=	1			;Transmit pin is PD1
TxPORT		=	_SFR_IO_ADDR(PORTD)
RxPORT		=	_SFR_IO_ADDR(PIND)

;***** Global register variables

;bitcnt		=	r16			;bit counter
;temp		=	r17			;temporary storage register

;Txbyte		=	r18			;Data to be transmitted
;Rxbyte		=	r19			;Received data

stopbits	=	1			;Number of stop bits (1, 2, ...)
;baudmulti	=	31			;19200 bps @ 4 MHz crystal
baudmulti	=	19			;115200 bps @ 16 MHz crystal
;baudmulti	=	8			;230400 bps @ 16 MHz crystal
;baudmulti	=	2			;460800 bps @ 16 MHz crystal


.section .text
;***************************************************************************
;*
;* "putchar"
;*
;* This subroutine transmits the byte stored in the "Txbyte" register
;* The number of stop bits used is set with the sb constant
;*
;* Number of words	:14 including return
;* Number of cycles	:Depens on bit rate
;* Low registers used	:None
;* High registers used	:2 (bitcnt,Txbyte)
;* Pointers used	:None
;*
;***************************************************************************
	.global uart_putc
	.func	uart_putc

uart_putc:	
	push	r16
	push	r17
	ldi	r16,9+stopbits	;1+8+sb (sb is # of stop bits)
	com	r24		;Inverte everything
	sec			;Start bit

uart_putc0:	
	brcc	uart_putc1	;If carry set
	cbi	TxPORT,TxD		; send a '0'
	rjmp	uart_putc2	;else	

uart_putc1:	
	sbi	TxPORT,TxD	; send a '1'
	nop

uart_putc2:	
	rcall	uart_delay	;One bit delay
	rcall	uart_delay

	lsr	r24		;Get next bit
	dec	r16		;If not all bit sent
	brne	uart_putc0	;   send next
					;else
	pop	r17
	pop	r16
	ret			;   return
	.endfunc


;***************************************************************************
;*
;* "getchar"
;*
;* This subroutine receives one byte and returns it in the "Rxbyte" register
;*
;* Number of words	:14 including return
;* Number of cycles	:Depens on when data arrives
;* Low registers used	:None
;* High registers used	:2 (bitcnt,Rxbyte)
;* Pointers used	:None
;*
;***************************************************************************
	.global	uart_getc
	.func	uart_getc

uart_getc:
	push	r16
	push	r17
	ldi 	r16,9	;8 data bit + 1 stop bit

uart_getc1:
	sbic 	RxPORT,RxD	;Wait for start bit
	rjmp 	uart_getc1

	rcall 	uart_delay	;0.5 bit delay

uart_getc2:
	rcall 	uart_delay	;1 bit delay
	rcall 	uart_delay		

	clc			;clear carry
	sbic 	RxPORT,RxD	;if RX pin high
	sec			;

	dec 	r16		;If bit is stop bit
	breq 	uart_getc3	;   return
					;else
	ror 	r24		;   shift bit into Rxbyte
	rjmp 	uart_getc2	;   go get next

uart_getc3:
	clr	r25
	pop	r17
	pop	r16	
	ret
	.endfunc


;***************************************************************************
;*
;* "UART_delay"
;*
;* This delay subroutine generates the required delay between the bits when
;* transmitting and receiving bytes. The total execution time is set by the
;* constant "b":
;*
;*	3·b + 7 cycles (including rcall and ret)
;*
;* Number of words	:4 including return
;* Low registers used	:None
;* High registers used	:1 (temp)
;* Pointers used	:None
;*
;***************************************************************************
; Some b values: 	(See also table in Appnote documentation)
;
; 1 MHz crystal:
;   9600 bps - b=14
;  19200 bps - b=5
;  28800 bps - b=2
;
; 2 MHz crystal:
;  19200 bps - b=14
;  28800 bps - b=8
;  57600 bps - b=2

; 4 MHz crystal:
;  19200 bps - b=31
;  28800 bps - b=19
;  57600 bps - b=8
; 115200 bps - b=2

uart_delay:	
	ldi	r17,baudmulti
uart_delay1:	
	dec	r17
	brne	uart_delay1
	ret
