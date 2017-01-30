#ifndef F_CPU
#warning "F_CPU was not defined before, will do now with 16000000"
#define F_CPU 16000000L    // system clock in Hz, "L" at the end is important, DO NOT use "UL"!
#endif

// Select soft uart for arbitrary avr pin usage
// (hardware uart is only available at dedicated pin locations)
#define USE_SOFTUART

//#define BAUD 500000L
//#define BAUD 460800L
//#define BAUD 230400L
#define BAUD 115200L
//#define BAUD 57600L
//#define BAUD 38400L
//#define BAUD 19200L

// Select size of sram circuit used [in bytes]
#define SRAMSIZE 32768L

// System includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// User includes
#include "crc16.h"


#ifdef USE_SOFTUART

    /* Define external uart functions
     * (external source is assembly for critical uart timing to match baudrate)
    */
    #include "uart.h"
    // Obsolete hardware init for soft uart
    #define init_uart() ;

#else

    // Calculations
    #define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)       // round cleverly
    #define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))         // real baudrate
    #define BAUD_ERROR ((BAUD_REAL*1000)/BAUD-1000)     // error per mill

    // Verify selected baudrate can be used with minimal error for the given CPU clock.
    #if ((BAUD_ERROR>10) || (BAUD_ERROR<-10))
    #error Systematic baudrate error is too high! (> 1%)
    #endif

    /*
     * initialize the avr
     */
    void init_uart(void){
        UCSRB |= (1<<RXEN)|(1<<TXEN);                   // enable uart rx+tx
        UCSRC |= (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);      // async 8N1

        UBRRH = UBRR_VAL >> 8;
        UBRRL = UBRR_VAL & 0xFF;
    }

    // status register of newer AVRs have different names, here for ATmega16:
    void uart_putc(unsigned char c)
    {
        while (!(UCSRA & (1 << UDRE)));     // wait until sending is possible
        UDR = c;                            // send character
    }

    /*
     * receive a char over UART
     */
    uint8_t uart_getc( void )
    {
        /* Wait for data to be received */
        while (!(UCSRA & (1 << RXC)));
        /* Get and return received data from buffer */
        return UDR;
    }

#endif


/*
 * start counting write cycles:
 * enable extern T1 pin, falling edge
 */
#define starttimer() TCCR1B = ((1 << WGM12) | (3 << CS11))

/*
 * stop counting write cycles
 */
#define stoptimer() TCCR1B = (1 << WGM12)

/* switch SRAM to WRITE mode */
#define sramwritemode() PORTB |= 1

/* switch SRAM to READ mode */
#define sramreadmode() PORTB &= ~1

/* set SRAM clock line (T) to HIGH */
#define clkhigh() PORTB |= (1 << 5)

/* set SRAM clock line (T) to LOW */
#define clklow() PORTB &= ~(1 << 5)

/* write 1 byte (8 bit) to external SRAM */
#define sramwrite (data) \
    PORTD = (PORTD & ~(0xFC)) | ((data & 0x3F) << 2); \
    PORTC = (PORTC & ~(0x03)) | ((data & 0xC0) >> 6);

/* read 1 byte (8 bit) from external SRAM */
#define sramread() ((PIND & 0xFC) >> 2) | ((PINC & 0x03) << 6)

/*
 * select samplerate for next snapshot:
 *
 * 0 -> 32.00Mhz  (24.00 Mhz)
 * 1 -> 16.00Mhz  (12.00 Mhz)
 * 2 ->  8.00Mhz  ( 6.00 Mhz)
 * 3 ->  4.00MHz  ( 3.00 Mhz)
 * 4 ->  2.00MHz  ( 1.50 Mhz)
 * 5 ->  1.00MHz  ( 0.75 Mhz)
 * 6 -> external
 * 7 -> avr (internal)
 */
#define setsamplerate(srate) PORTB = (PORTB & ~(0x07 << 2)) | ((srate & 0x07) << 2)



// recording state (1 = on, 0 = off),
// volatile because it's used in interrupt routine
volatile uint8_t recording = 0;



void dataoutmode(void)
{
    /* PORT D:
        Bit0: UART-RX, Bit1: UART-TX, Bit2-7: D0-D5 */
    DDRD = 0xFE;
    PORTD = 0xFD;
    /* PORT C:
        Bit0-1: D6-D7 */
    DDRC = 0xFF;
    PORTC = 0xFF;
}


void datainmode(void)
{
    /* PORT D:
        Bit0: UART-RX, Bit1: UART-TX, Bit2-7: D0-D5 */
    DDRD = 0x02;
    PORTD = 0xFD;
    /* PORT C:
        Bit0-1: D6-D7 */
    DDRC = 0xFC;
    PORTC = 0xFF;
}


void init (void)
{
    DDRA = 0xFF;
    PORTA = 0xFF;
    DDRB = 0xFD;
    PORTB = 0x3C;
}


/*
 * set the counter
 */
void init_count(void){
    TCCR1A = 0;
    TCCR1B = (1 << WGM12);
    TCNT1 = 0;
    OCR1A = 15;     // count 15 falling edges of A10 line (=32k memory, 17k pretrigger?)
    TIMSK |= (1 << OCIE1A);
}


/*
 * Long variable delay time, unit in ms.
 *
 * Maximum time per function call is limitted to 262.14 ms / F_CPU in MHz.
 *
 * For this the waiting loop function is called several times to get longer delays.
 * The additional checking of the loop condition gives a little inaccuracy in
 * delay time (about 2 to 3 ms).
 *
*/
void long_delay(uint16_t ms) {
    for(; ms>0; ms--) _delay_ms(1);
}



/*
 * main loop
 */
int main(void)
{
    uint8_t trig, samp, c;
    uint8_t i;
    uint16_t j;

    // Initialize system (start condition)
    init();
    datainmode();
    init_uart();

    _delay_ms(1);

    sei();

    while(1) {

        while(uart_getc() != 'L') {};
        trig = uart_getc();  //trigger
        samp = uart_getc();  //samplerate

        setsamplerate(samp);
        init_count();
        sramwritemode();
        recording = 1;
        //Wait until SRAM is written completely at least 1 time.
        //Max. duration: 32ms for 1MHz = 32000 Bytes written.
        long_delay(40);

        // Trigger: rising, falling, no
        switch(trig) {
            case 'r':
                //INT0 clear
                GIFR = 0x40;
                //INT0 rising edge trigger
                MCUCR = (MCUCR & 0xF0) | 0x03;
                //INT0 enable
                GICR |= (1 << INT0);
                break;
            case 'f':
                //INT0 clear
                GIFR = 0x40;
                //INT0 falling edge trigger
                MCUCR = (MCUCR & 0xF0) | 0x02;
                //INT0 enable
                GICR |= (1 << INT0);
                break;
            default:
                starttimer();
        }

        // wait until snapshot recording is done
        while(recording){}

        // output to PC and calculate CRC16 for transport protection
        uart_putc('K');
        // transmit single blocks with blocksize=8192
        // 8192 = max. size to detect single bit errors
        for(i = 0; i < (SRAMSIZE / 8192); i++){
            // crc init value
            crc16 = 0xffff;;
            for (j = 0; j < 8192; j++){
                clkhigh();
                clklow(); // clock next value
                c = sramread();
                calc_crc16(c);
                uart_putc(c);
            }
            // transmit crc16 after each block
            uart_putc(crc16&0xff);
            uart_putc(crc16>>8);
        }
    }

    return 0;
}

/*
 * Gets called when "snapshot done" condition occurs (=watch SRAM fill level) -> stop recording.
 * (Timer ISR gets called after A10 memory address line was triggered selected number of times.
 *  By changing OCR1A register value (=number of A10 triggers counted)
 *  -> pre-trigger and post-trigger length can be selected.)
 */
ISR(TIMER1_COMPA_vect)
{
    setsamplerate(7);   // select manual avr sample clock (stop sampling)
    sramreadmode();     // switch SRAM to read mode
    stoptimer();        // stop timer
    recording = 0;      // stop snapshot recording
}

/*
 * Trigger condition found
 * -> disable INT0 (no re-trigger allowed)
 * -> start counting SRAM fill level (stop when full)
 */
ISR(INT0_vect)
{
    GICR &= ~(1 << INT0);
    starttimer();
}
