#ifndef UART_H
#define UART_H

#ifndef F_CPU
#define F_CPU 4000000UL
#endif


#include <inttypes.h>

#define BAUD 9600UL

// calculate baud
#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)   
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))   
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD) 


#if ((BAUD_ERROR<990) || (BAUD_ERROR>1010))
#error Error in baudrate > 0.1
#endif

#define UART_BUFFER_SIZE 80

#define UART_ECHO 0

// Functions 
void USART_init();
void USART_transmit( unsigned char data );
void USART_puts( char*s );
void USART_writeln( char* s );

char* USART_has_data();

#endif

