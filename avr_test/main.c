
#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "uart.h"

uint16_t cap_sense;

#define STATUS_PORT PORTB      
#define STATUS_DDR  DDRB       
#define STATUS_LED  (1<<PB0)   


volatile uint16_t cnt;


int main()
{
	USART_init();

  STATUS_DDR |= STATUS_LED; 

  // print welcome 
  USART_writeln( "100 AVR Testing 0.1.1 (c) 2013 Matthias Hannig" );

	for(;;) {
    STATUS_PORT ^= STATUS_LED;
    _delay_ms(500);

    USART_writeln( "Hi there!");
	}	
}


