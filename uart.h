// uart.h
// PJ, 2023-12-01
//     2025-02-22 trimmed code for use in NPP-301 characterizer

#ifndef MY_UART
#define MY_UART
void uart1_init(long baud);
void uart1_putch(char data);
void uart1_putstr(char* str);
void uart1_flush_rx(void);
char uart1_getch(void);
int uart1_getstr(char* buf, int nbuf);
void uart1_close(void);

void putch(char data);
int getch(void);
int getche(void);

#define XON 0x11
#define XOFF 0x13

#endif
