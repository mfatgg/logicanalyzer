;******* CRC16 calculation in efficient assembly *******
;*
;* - manually converted from C source
;* - save registers for clean C-callable function
;* - use 0xA001 as crc polynom
;*   (=reverse CRC-16 polynom: x^16 + x^15 + x^2 + 1)
;*******************************************************

.section .data

    .global crc16
crc16:
crc_lo:
    .byte 0
crc_hi:
    .byte 0


.section .text

    .global calc_crc16
    .func   calc_crc16
calc_crc16:
    push    r16
    push    r17
    push    r18
    push    r19
    lds     r16, crc_lo
    lds     r17, crc_hi
    ldi     r18, 0x01
    ldi     r19, 0xA0

    eor     r16, r24
    ldi     r24, 8
_crc1:
    lsr     r17
    ror     r16
    brcc    _crc2
    eor     r16, r18
    eor     r17, r19
_crc2:
    dec     r24
    brne    _crc1

    sts     crc_hi, r17
    sts     crc_lo, r16
    pop     r19
    pop     r18
    pop     r17
    pop     r16
    ret
    .endfunc
