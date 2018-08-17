#include <console.h>
#include <stdarg.h>
#include "typedefs.h"
#include "printf.h"
#include "BSP.h"

/*-------------------------------------------------------------------*/
static void outbyte(unsigned char c) 
{
	UART_Send(c);
}
void putchar1(unsigned char c) 
{
    (void)c;
    RS485_DEBUG_SEND_ENABLE;
    UART_Send(c);
    RS485_DEBUG_RECV_ENABLE;
}
void putchar2(unsigned char c) 
{	
    UART_Send(c);
}
/*-------------------------------------------------------------------*/

//static void printchar(char **str, long c)
static void printchar(void *pBuf, long c)
{
	/*
	if (str) 
	{
		**str = c;
		++(*str);
	}
	else 
	{
		(void)outbyte(c);
	}
	*/

	if (pBuf) 
	{
		//RBuffer_Write1((SRingBuffer*)pBuf, (uint8_t*)&c, 1);	//safe if pBuf is full
	}
	else 
	{
		(void)outbyte(c);
	}

}
/*-------------------------------------------------------------------*/
#define PAD_RIGHT 1
#define PAD_ZERO  2

//static long prints(char **out, const char *string, long width, long pad)
static long prints(void *out, const char *string, long width, long pad)
{
	register long pc = 0, padchar = ' ';

	if (width > 0) {
		register long len = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len;
		if (len >= width) width = 0;
		else width -= len;
		if (pad & PAD_ZERO) padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			printchar (out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) {
		printchar (out, *string);
		++pc;
	}
	for ( ; width > 0; --width) {
		printchar (out, padchar);
		++pc;
	}

	return pc;
}
/*-------------------------------------------------------------------*/
/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

//static long printi(char **out, long i, long b, long sg, long width, long pad, long letbase)
static long printi(void *out, long i, long b, long sg, long width, long pad, long letbase)
{
	char print_buf[PRINT_BUF_LEN];
	register char *s;
	register long t;
	register long neg = 0, pc = 0;
	register unsigned long u = i;

	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints (out, print_buf, width, pad);
	}

	if (sg && b == 10 && i < 0) {
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) {
		t = u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;
	}

	if (neg) {
		if( width && (pad & PAD_ZERO) ) {
			printchar (out, '-');
			++pc;
			--width;
		}
		else {
			*--s = '-';
		}
	}

	return pc + prints (out, s, width, pad);
}
/*-------------------------------------------------------------------*/
//static long print(char **out, const char *format, va_list args )
static long print(void *out, const char *format, va_list args )
{
	register long width, pad;
	register long pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) {
				register char *s = (char*)va_arg( args, long );
				pc += prints (out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) {
				pc += printi (out, va_arg( args, long ), 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) {
				pc += printi (out, va_arg( args, long ), 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				pc += printi (out, va_arg( args, long ), 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) {
				pc += printi (out, va_arg( args, long ), 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char)va_arg( args, long );
				scr[1] = '\0';
				pc += prints (out, scr, width, pad);
				continue;
			}
		}
		else {
		out:
			printchar (out, *format);
			++pc;
		}
	}
	//if (out) **out = '\0';
	
	va_end( args );
	
	
	return pc;
}

/*-------------------------------------------------------------------*/
void uart_printf(const char *format, ...)
{
	RS485_DEBUG_SEND_ENABLE;
	
	va_list args;

	va_start( args, format );

	print( 0, format, args );
	
	RS485_DEBUG_RECV_ENABLE;      
}
/*-------------------------------------------------------------------*/
//int buf_sprintf(char *out, const char *format, ...)
int buf_sprintf(void *out, const char *format, ...)
{
	
	va_list args;

	va_start( args, format );

	//print( &out, format, args );
	print( out, format, args );
	

	return 0;
        
}

/*-------------------------------------------------------------------*/
