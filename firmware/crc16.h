#ifndef _CRC16_H
#define _CRC16_H

#include <avr/io.h>

/* current crc value */
extern volatile uint16_t crc16;

/* calc next crc value for character given */
extern void calc_crc16 (unsigned char c);

#endif
