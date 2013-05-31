
#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "uart.h"

uint16_t cap_sense;

#define STATUS_PORT PORTC      
#define STATUS_DDR  DDRC       
#define STATUS_LED  (1<<PC3)   


#define SENS_PORT    PORTD
#define SENS_DDR     DDRD
#define SENS_TX_PIN  (1<<PD3)
#define SENS_RX_PIN  (1<<PD2)


int main()
{
  char  buffer[80];
  char* data;

	// wait
	USART_init();

  uint8_t note;
  uint8_t velocity;
  

  // Enable ext interrupt
  /*
  GICR  |= 1<<INT0; 
  MCUCR |= 1<<ISC01 | 1<<ISC00; // Trigger on rising edge.
  */


  sei();



	for(;;) {

    // Play tone 
    note = 32 + (rand() % 64);
    velocity = 40 + (rand() % 90 );      
      
    // Note ON:
    USART_transmit( 0x90 );
    USART_transmit( note );
    USART_transmit( velocity );    

    _delay_ms( 200 );
    USART_transmit( 0x80 );
    USART_transmit( note );
    USART_transmit( 0x00 );
    
    _delay_ms( 50 );

    
	}	
}


