#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "readargs.h"


// common constants
#define FALSE   0
#define TRUE    1
// external sram size is 32kbytes
#define SRAMSIZE 32768


// define helper and serial (com port) functions
void helptext(void);
void com_open(char device[], speed_t baud);
void com_close(void);
void com_putc(unsigned char c);
int  com_getc(int timeout);
void com_puts(char *text);


// enable debug output
#undef  DEBUG
#define DEBUG

// select baurate to use
//
//#define BAUD_DEF  13       //19200
//#define BAUD_DEF  14       //38400
//#define BAUD_DEF  15       //57600
//#define BAUD_DEF  16       //115200
#define BAUD_DEF    17       //230400
//#define BAUD_DEF  18       //460800
//#define BAUD_DEF  19       //500000

#define DEVICE "/dev/ttyUSB0"       //Default Device

char device[] = DEVICE;

speed_t         baud_const[] = {
                    B50, B75, B110, B134, B150, B200, B300, B600, B1200, B1800, B2400, B4800,
                    B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B1000000
                };

unsigned long   baud_value[] = {
                    50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800,
                    9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 1000000
                };

int             baudrate = BAUD_DEF;

struct          termios  oldtio;
int             fd;                 // Serial device
int             _argc;
char            **_argv;
int             in=-1;
unsigned int    crc16, crc16_received[SRAMSIZE/8192];


/*
 * Update output file (VCD) with next data.
 */
void update(FILE* f, long time, char data, char diff)
{
    int i;

    fprintf(f,"#%ld\n",time);
    for(i=0; i < 8; i++){
        if(diff & (1 << i)){
            fprintf(f, "%d%c\n", (data >> i) & 1, 'h'-i);
        }
    }
}


/*
 * Help text when no / wrong command line is given.
 */
void helptext(void)
 {
    printf(
        "\n\t /?            \t\t Get this help message\n"
        "\t /Bnnnn        \t\t Define baud rate\n"
        "\t /Ddevice      \t\t Define serial port\n"
        "\n\n"
        // "\t /D         \t\t Debug Mode\n\n"
    );
    exit(1);
}


/*
 * Print given "number" in binary format.
 */
void printbin8(int number)
{
    int i;
    for (i=0; i < 8; i++)
    {
        if (i == 5)
        {
            if (number & 0x80)
                printf("%c", '1');
            else
                printf("%c", '0');
        }
        number = (number << 1);
    }
}


/*
 * Calculate next CRC-16 value for character "c".
 * CRC polynom used is 0xA001 (=reverse x^16 + x^15 + x^2 + 1)
 */
void calc_crc16(unsigned char c) {
    int i;

    crc16 ^= c;
    for (i = 8; i; i--) {
        crc16 = (crc16 >> 1) ^ ((crc16 & 1) ? 0xA001 : 0 );
    }
}


/*
 * Main function to read sample snapshot from firmware (hardware) and
 * convert it to .VCD output file.
 * (so logic analyzer data can later be viewed by GTKwave)
 */
int main( int argc, char *argv[])
 {
    int             success;
    unsigned long   tmp_baud;
    char            s[32];
    unsigned char   buf[SRAMSIZE];
    unsigned int    i,k;
    FILE            *f = NULL;
    char            *vcdfile;
    char            j, diff;
    int             noisecancel = 0;
    float           step;

    _argc = argc;
    _argv = argv;

    // No / wrong command line given?
    if (argc < 2 || readargs( ABOOL, '?', &i))
        helptext();

    // Check if baudrate is valid
    if (readargs(ALONG, 'b', &tmp_baud)) {
        baudrate = -1;

        i = 0;
        while(i < sizeof(baud_value) / sizeof(baud_value[0])) {
            if (baud_value[i] == tmp_baud) {
                baudrate = i;
                break;
            } else {
                i++;
            }
        }
        if (baudrate < 0 ) {
            printf("Ungültige Baudrate!");
            helptext();
        }
    }

    // Read device string
    if (readargs( ASTRING, 'd', &s) )
        strcpy(device, s);

    // Output status info
    printf ("using Device: %s\n", device);
    printf ("using Baudrate: %lu\n", baud_value[baudrate]);

    // Init serial com port
    com_open(device, baud_const[baudrate]);
    if (fd < 0) { perror(device); exit(-1); }

    // Main loop
    success = 0;
    while (!success) {

        printf("Starting µC\n");
        //tcflush(fd, TCIOFLUSH);
        // Fire acquisition state machine in firmware
        com_putc ('L');
        //tcflush(fd, TCOFLUSH);
        // Trigger: raising 'r', falling 'f', no 'other'
        com_putc (' ');
        //tcflush(fd, TCOFLUSH);
        // Samplerate: 0-5=int(32Mhz-1Mhz), 6=ext
        com_putc (1);
        step = 1000 / 16; //16 Mhz Samplerate
        //tcflush(fd, TCOFLUSH);
        //tcdrain(fd);

        printf("Trying to receive\n");
        in = com_getc (1);
        printf("%02x ", in);

        // Wait for snapshot transmission by firmware,
        // character 'K' signals start of transmission.
        if (in == 'K') {
            printf("Got answer from µC!\n");

            // Receive all SRAM blocks at first
            for (k = 0; k < (SRAMSIZE / 8192); k++) {
                i = 0;
                while (i<8192) {
                    i += read(fd, &buf[8192*k+i], 8192-i);
                    //printf ("%lu ", i);
                }
                crc16_received[k] = (com_getc(1) & 0xff) | (com_getc(1) << 8);
            }

            // Verify CRC of all SRAM blocks after reception
            for (k = 0; k < (SRAMSIZE / 8192); k++) {

                // Calculate CRC16
                crc16 = 0xffff;
                for (i = 0; i < 8192; i++) {
                    calc_crc16(buf[8192 * k + i]);
                }
                printf("calculated [%u] crc16: %04x\n", k, crc16);
                printf("received   [%u] crc16: %04x\n", k, crc16_received[k]);
                if (crc16 == crc16_received[k])
                    printf("SUCCESS: CRCs are equal!\n");
                else
                    printf("ERROR: CRCs are NOT equal!!!\n");
            }

            // Set exit condition for main loop (=1 snapshot received)
            success = 1;

            // Create VCD file
            f = fopen("test.vcd","w");
            // Write VCD header
            fprintf(f, "$date\n");
            fprintf(f, "Thu May 24 17:15:31 2007\n");
            fprintf(f, "$end\n");
            fprintf(f, "$version\n");
            fprintf(f, "LAdump 1.0\n");
            fprintf(f, "$end\n");
            fprintf(f, "$timescale\n");
            fprintf(f, "1ns\n");
            fprintf(f, "$end\n");
            fprintf(f, "$scope module benchx $end\n");
            fprintf(f, "$var wire 1 a D7 $end\n");
            fprintf(f, "$var wire 1 b D6 $end\n");
            fprintf(f, "$var wire 1 c D5 $end\n");
            fprintf(f, "$var wire 1 d D4 $end\n");
            fprintf(f, "$var wire 1 e D3 $end\n");
            fprintf(f, "$var wire 1 f D2 $end\n");
            fprintf(f, "$var wire 1 g D1 $end\n");
            fprintf(f, "$var wire 1 h D0 $end\n");
            fprintf(f, "$upscope $end\n");
            fprintf(f, "$enddefinitions $end\n");
            fprintf(f, "$dumpvars\n");
            // Output first VCD data (init)
            update(f, 0, buf[0], 0xff);
            // Loop over full sample snapshot and output data to VCD file
            for (i=1; i < (SRAMSIZE-1); i++) {
                if ((diff = (buf[i - 1] ^ buf[i])) != 0) {
                    for (j=1; (j <= noisecancel) && (i+j <= (SRAMSIZE-1)); j++)
                        diff &= ~(buf[i] ^ buf[i+j]);
                    update(f, i * step, buf[i], diff);
                }
            }
            update(f, (SRAMSIZE-1) * step, buf[SRAMSIZE-1], 0xff);
            fclose(f);

        } else {

            // Save shutdown in error case
            tcflush(fd, TCIOFLUSH);
            com_close();
            com_open(device, baud_const[baudrate]);

        }
    }

    // Close serial port after communication is done
    com_close();

    return 0;
}



/*
 * Functions for serial communication following.
 * TODO: move to separate library.
 */

/*
 * Open serial communication port.
 * \param device string for serial device (e.g. /dev/ttyUSB0 for usb->serial convert on Linux)
 * \param baud baudrate selection
 */
void com_open(char device[], speed_t baud) {
    struct termios newtio;

    // Open device
    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        return;
    }

    // Store settings
    tcgetattr(fd, &oldtio);

    // Init struct with zeros
    memset(&newtio, 0x00 , sizeof(newtio));

    // Set flags
    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | IGNBRK;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;

    //cfmakeraw(&newtio);

    // Timeout in 100ms
    //newtio.c_cc[VTIME] = 0;
    // Read 1 character at a time
    //newtio.c_cc[VMIN] = 0;

    // Set baudrate
    cfsetispeed(&newtio, baud);
    cfsetospeed(&newtio, baud);

    // Flush buffers
    //tcflush(fd, TCIOFLUSH);

    // Set new port settings
    tcsetattr(fd, TCSANOW, &newtio);

    tcflush(fd, TCIOFLUSH);
    return;
}


/*
 * Close serial communication port (and restore previous serial state).
 */
void com_close(void) {
    // Restore old settings
    tcsetattr(fd, TCSANOW, &oldtio);

    // Close device
    close(fd);
}


/*
 * Put 1 character on serial line (and flush output).
 * \param c character to send
 */
void com_putc(unsigned char c) {
    write(fd, &c, 1);
    tcdrain(fd);
}


/*
 * receive 1 character from serial line
 * \param timeout define serial timeout in seconds
 * \return character received or -1 in timeout case
 */
int com_getc(int timeout)
 {
    unsigned char c= 0;
    clock_t       t= clock();

    do {
        if (read(fd, &c, 1) == 1)
            return c;
    while ((clock() - t) / CLOCKS_PER_SEC < timeout);

    return -1;
 }

/*
 * put string on serial line
 * \param text = c-string
 */
void com_puts(char *text) {
    while (*text)
        com_putc(*text++);
}
