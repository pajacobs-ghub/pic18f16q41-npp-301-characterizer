// npp301-characterizer.c
// Use a PIC18F16Q41 to supply the NPP-301 sensor package with
// an excitation voltage and then report the voltages
// at the intermediate pins of the NPP-301 package.
//
// PJ, 2025-02-22: Basic command interpreter.
//
// CONFIG1
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_64MHZ

// CONFIG2
#pragma config CLKOUTEN = OFF
#pragma config PR1WAY = OFF
#pragma config CSWEN = OFF
#pragma config FCMEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF

// CONFIG3
#pragma config MCLRE = EXTMCLR
#pragma config PWRTS = PWRT_64
#pragma config MVECEN = OFF
#pragma config IVT1WAY = OFF
#pragma config LPBOREN = OFF
#pragma config BOREN = SBORDIS

// CONFIG4
#pragma config BORV = VBOR_1P9
#pragma config ZCD = OFF
#pragma config PPS1WAY = OFF
#pragma config STVREN = ON
#pragma config LVP = ON
#pragma config XINST = OFF

// CONFIG5
#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = ON

// CONFIG6
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC

// CONFIG7
#pragma config BBSIZE = BBSIZE_512
#pragma config BBEN = OFF
#pragma config SAFEN = OFF
#pragma config DEBUG = OFF

// CONFIG8
#pragma config WRTB = OFF
#pragma config WRTC = OFF
#pragma config WRTD = OFF
#pragma config WRTSAF = OFF
#pragma config WRTAPP = OFF

// CONFIG9
#pragma config CP = OFF

#include <xc.h>
#include "global_defs.h"
#include <stdint.h>
#include <stdlib.h>

#include "uart.h"
#include <stdio.h>
#include <string.h>

#define VERSION_STR "0.1 PIC18F16Q41 NPP-301 Characterizer"

// Each device on the RS485 network has a unique single-character identity.
// The master (PC) has identity '0'. Slave nodes may be 1-9A-Za-z.
// When programming each device, select a suitable value for MYID.
#define MYID 'N'

#define GREENLED (LATCbits.LATC4)
uint8_t override_led = 0;

void init_pins()
{
    // RC4 as digital-output for GREENLED.
    TRISCbits.TRISC4 = 0;
    GREENLED = 0;
    //
}

void FVR_init()
{
    // We want to supply both the ADC and the DAC with 4V.
    FVRCONbits.ADFVR = 3;  // 4v096
    FVRCONbits.CDAFVR = 3; // 4v096
    FVRCONbits.EN = 1;
    while (!FVRCONbits.RDY) { /* should be less than 25 microseconds */ }
    return;
}

void FVR_close()
{
    FVRCONbits.EN = 0;
    FVRCONbits.ADFVR = 0;
    FVRCONbits.CDAFVR = 0;
    return;
}

void set_VREF_on(uint8_t level)
{
    // Assuming that the fixed voltage reference is on at 4v096,
    // take a fraction of that voltage and feed it through the
    // DAC1 and then through the OPA1 to the external pin (OPA1OUT/RC2).
    //
    DAC1CONbits.PSS = 0b10; // FVR Buffer 2
    DAC1CONbits.NSS = 0; // VSS
    DAC1CONbits.EN = 1;
    DAC1DATL = level;
    //
    OPA1CON2bits.PCH = 0b100; // DAC1_OUT
    OPA1CON0bits.UG = 1; // unity gain
    OPA1CON0bits.CPON = 1; // charge pump active
    OPA1CON0bits.SOC = 0; // basic operation
    OPA1CON0bits.EN = 1;
    return;
}

void set_VREF_off()
{
    OPA1CON0bits.EN = 0;
    DAC1CONbits.EN = 0;
    return;
}

#define MY_ANC2 0b00010010
#define MY_ANC7 0b00010111
#define MY_ANB5 0b00001101
#define MY_ANB6 0b00001110
#define MY_ANB7 0b00001111

void ADC_init()
{
    // Set up the ADC to look at the op-amp output.
    TRISCbits.TRISC2 = 1;
    ANSELCbits.ANSELC2 = 1;
    //
    ADCON0bits.CS = 1; // use dedicated RC oscillator, T_AD = 2us
    ADCON0bits.FM = 1; // right-justified result
    ADCON2bits.MD = 0b011; // burst average mode
    ADCON2bits.CRS = 4; // divide accumulated result by 16
    ADRPT = 16; // number of repeated triggers
    ADACQ = 50; // 100us acquisition time
    PIR1bits.ADIF = 0;
    ADREFbits.NREF = 0; // negative reference is Vss
    ADREFbits.PREF = 0b11; // positive reference is FVR
    ADACQ = 0x10; // 16TAD acquisition period
    ADPCH = MY_ANC2; // select ANC2/RC2
    ADCON0bits.ON = 1; // power on the device

    return;
}

uint16_t ADC_read(uint8_t ain)
{
    // Returns the value from the ADC when looking at the selected pin.
    ADPCH = ain;
    ADCON0bits.GO = 1;
    NOP();
    while (ADCON0bits.GO) { /* wait, should be brief */ }
    PIR1bits.ADIF = 0;
    return ADRES;
}

void ADC_close()
{
    ADCON0bits.ON = 0;
    return;
}

// For incoming UART comms
#define NBUFA 80
char bufA[NBUFA];
// For outgoing UART comms
#define NBUFB 268
char bufB[NBUFB];

int find_char(char* buf, int start, int end, char c)
// Returns the index of the character if found, -1 otherwise.
// start is the index of the first character to check.
// end is the index of the last character to check.
{
    for (int i = start; i <= end; i++) {
        if (buf[i] == '\0') return -1;
        if (buf[i] == c) return i;
    }
    return -1;
}

char* trim_RS485_command(char* buf, int nbytes)
// Returns a pointer to the command text string, within buf.
// The resulting string may be zero-length.
//
// A valid incoming command from the RS485 will be of the form
// "/cXXXXXXXX!"
// where the components are
//    / is the start character
//    ! is the end character
//    c is the MYID character, identifying the receiving node
//    XXXXXXX is the command text
//
// This format is described in the text:
// J.M. Hughes
// Real World Instrumentation
// O'Rielly 2010
// Chapter 11 Instrumentation Data I/O, Unique Protocols.
//
{
    // printf("DEBUG: buf=%s", buf);
    int start = find_char(buf, 0, nbytes-1, '/');
    if (start == -1) {
        // Did not find starting '/'
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    int end = find_char(buf, start, nbytes-1, '!');
    if (end == -1) {
        // Did not find terminating '!'
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    // At this point, we have a valid incoming command.
    if (buf[start+1] != MYID) {
        // The incoming message is not for this node, so discard it.
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    // At this point, the message is for us.
    buf[end] = '\0'; // Trim off the '!' character.
    // On return, omit the MYID character from the front.
    return &buf[start+2];
} // end trim_command()

void interpret_RS485_command(char* cmdStr)
// A command that does not do what is expected should return a message
// that includes the word "error".
{
    char* token_ptr;
    const char* sep_tok = ", ";
    int nchar;
    uint8_t i, j;
    char number_str[10];
    // nchar = printf("DEBUG: cmdStr=%s", cmdStr);
    if (!override_led) GREENLED = 1; // To indicate start of interpreter activity.
    switch (cmdStr[0]) {
        case 'v':
            // Echo the version string for the PIC18 firmware.
            nchar = snprintf(bufB, NBUFB, "/0v %s#\n", VERSION_STR);
            uart1_putstr(bufB);
            break;
        case 'L':
            // Turn LED on or off.
            // Turning the LED on by command overrides its use
            // as an indicator of interpreter activity.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume on/off value.
                // Use just the least-significant bit.
                i = (uint8_t) (atoi(token_ptr) & 1);
                GREENLED = i;
                override_led = i;
                nchar = snprintf(bufB, NBUFB, "/0L %d#\n", i);
            } else {
                // There was no text to give a value.
                nchar = snprintf(bufB, NBUFB, "/0L error: no value#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'a': {
            // Report the ADC value for the analog signal on the comparator input.
            uint16_t pin8_npp301 = ADC_read(MY_ANC2);
            uint16_t pin2_npp301 = ADC_read(MY_ANC7);
            uint16_t pin4_npp301 = ADC_read(MY_ANB7);
            uint16_t pin5_npp301 = ADC_read(MY_ANB6);
            uint16_t pin6_npp301 = ADC_read(MY_ANB5);
            nchar = snprintf(bufB, NBUFB, "/0a %u %u %u %u %u#\n", pin8_npp301,
                    pin2_npp301, pin4_npp301, pin5_npp301, pin6_npp301);
            uart1_putstr(bufB); }
            break;
        case 'w':
            // Enable VREF output, to feed an external ADC chip on the DAC-MCU.
            // Note that this is not relevant to all ADC MCUs.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume level followed by on/off flag.
                // The on/off flag is optional and defaults to 1.
                int16_t level = atoi(token_ptr);
                int8_t onOffFlag = 1;
                token_ptr = strtok(NULL, sep_tok);
                if (token_ptr) {
                    onOffFlag = (int8_t) atoi(token_ptr);
                }
                if (onOffFlag) {
                    if (level > 255) level = 255;
                    if (level < 0) level = 0;
                    set_VREF_on((uint8_t)level);
                    nchar = snprintf(bufB, NBUFB, "/0w VREF on level=%d#\n", level);
                } else {
                    set_VREF_off();
                    nchar = snprintf(bufB, NBUFB, "/0w VREF off#\n");
                }
            } else {
                // There was no text to indicate action.
                nchar = snprintf(bufB, NBUFB, "/0w error: missing level and on/off flag#\n");
            }
            uart1_putstr(bufB);
            break;
        default:
            nchar = snprintf(bufB, NBUFB, "/0%c error: Unknown command#\n", cmdStr[0]);
            uart1_putstr(bufB);
    }
    if (!override_led) GREENLED = 0; // To indicate end of interpreter activity.    
} // end interpret_RS485_command()

int main(void)
{
    int m;
    int n;
    init_pins();
    uart1_init(115200);
    FVR_init();
    ADC_init();
    __delay_ms(10);
    // Flash LED twice at start-up to indicate that the MCU is ready.
    for (int8_t i=0; i < 2; ++i) {
        GREENLED = 1;
        __delay_ms(250);
        GREENLED = 0;
        __delay_ms(250);
    }
    // Wait until we are reasonably sure that the AVR has restarted.
    __delay_ms(100);
    // We wait for commands and only responding when spoken to.
    while (1) {
        // Characters are not echoed as they are typed.
        // Backspace deleting is allowed.
        // NL (Ctrl-J) signals end of incoming string.
        m = uart1_getstr(bufA, NBUFA);
        if (m > 0) {
            char* cmd = trim_RS485_command(bufA, NBUFA);
            // Note that the cmd string may be of zero length,
            // with the null character in the first place.
            // If that is the case, do nothing with it.
            if (*cmd) {
                interpret_RS485_command(cmd);
            }
        }
    }
    uart1_flush_rx();
    uart1_close();
    return 0; // Expect that the MCU will reset.
} // end main
