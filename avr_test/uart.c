
#include "uart.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

volatile uint8_t uart_string_ready = 0;
volatile uint8_t uart_string_length = 0;
volatile char    uart_string_buffer[UART_BUFFER_SIZE] = "";

/**
 * UART receive interrupt
 */
ISR( USART_RXC_vect )
{
    static uint8_t count;
    unsigned char c = UDR;

    if( uart_string_ready == 0 ) {
		if( c == '\r' ) {
			// discard char
		}
        else if( c != '\n' &&
            count < UART_BUFFER_SIZE - 1 ) {
            // append char to string
            uart_string_buffer[count] = c;
            count++;
        }
        else {
            // terminate string
            uart_string_buffer[count] = '\0';
            uart_string_length = count;
            count = 0;
            uart_string_ready = 1;
        }
    }

// echo back
#if UART_ECHO == 1
    UDR = c;
#endif

}


void USART_transmit( unsigned char data )
{

    /* Wait for empty transmit buffer */
    while ( !( UCSRA & (1<<UDRE)) ) {
        // do_nothing() tm
    };

    /* Put data into buffer, sends the data */
    UDR = data;
}

void USART_puts( char *s )
{
    while (*s)
    {
        USART_transmit(*s);
        s++;
    }
}

void USART_writeln( char *s )
{
    USART_puts( s );
    USART_transmit( '\n' );
    USART_transmit( '\r' );
}

void USART_init()
{

    /* Set baud rate */
    UBRRH = (unsigned char)(UBRR_VAL>>8);
    UBRRL = (unsigned char)(UBRR_VAL & 0xff);

    UCSRC=(1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|
        (0<<USBS)|(0<<UCSZ2)|(1<<UCSZ1)|(1<<UCSZ0);

    /* Enable receiver and transmitter */
    // UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
    UCSRB |= (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);

    /* Set frame format: 8data, 2stop bit
     * and use interrupts for receiving data
     */
    // UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0)
    // UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
}

char* USART_has_data() 
{
	if( uart_string_ready == 1 ) {
		uart_string_ready = 0;
		return (char*)uart_string_buffer; 
	}

	return NULL;
}


