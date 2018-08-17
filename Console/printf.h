#ifndef _PRINTF_H_
#define _PRINTF_H_

void putchar1(unsigned char c);
void putchar2(unsigned char c); 

void uart_printf(const char *format, ...);
int buf_sprintf(void *out, const char *format, ...);

#endif
