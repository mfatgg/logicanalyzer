#include "crc16.h"

/* current crc value */
volatile uint16_t crc16;

/*
 * Calc next crc value for character given.
 * Use CRC-16 polynom 0xA001 (= reverse x^16 + x^15 + x^2 + 1, x^16 is implicit)
 */
void calc_crc16(unsigned char c)
{
    int i;

    crc16 ^= c;
    for(i = 8; i; i--) {
        crc16 = (crc16 >> 1) ^ ((crc16 & 1) ? 0xA001 : 0 );
    }
}
