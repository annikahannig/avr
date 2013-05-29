
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


// extern interrupt test
ISR( INT0_vect ) 
{
  // toggle led
  STATUS_PORT ^= STATUS_LED;
  
  cap_sense = TCNT1;
  
  // reset sens (pull to ground)
  SENS_DDR  |= SENS_RX_PIN;
  SENS_PORT &= ~SENS_RX_PIN;
  _delay_ms(1);
  SENS_DDR  &= ~SENS_RX_PIN; // 
  TCNT1 = 0; // reset timer;
}



int main()
{
  char  buffer[80];
  char* data;
  uint16_t value;
  uint8_t cmd;
  uint8_t cnt;

  cap_sense = 0;
  cnt = 0;

	// wait
	USART_init();

  STATUS_DDR |= STATUS_LED; 


  // initialize
  SENS_DDR   |= SENS_TX_PIN|SENS_RX_PIN; // 
  SENS_PORT  &= ~SENS_TX_PIN;
  SENS_PORT  &= ~SENS_RX_PIN; // PULL port down. clear capacity.

  _delay_ms( 10 );


  SENS_DDR &= ~SENS_RX_PIN; // use RX as input
  

  // Enable ext interrupt
  GICR  |= 1<<INT0; 
  MCUCR |= 1<<ISC01 | 1<<ISC00; // Trigger on rising edge.

  sei();




  // print welcome 
  USART_writeln( "100 Capsense Testing 0.1.1 (c) 2013 Matthias Hannig" );

  // setup timer
  TCCR1A = 0;
  TCCR1B = (1<<CS10); //|(1<<CS10); // Clear on 0xffff, Prescale: 64

  sei();

	for(;;) {

  
    // push charge to sensor
    SENS_PORT |= SENS_TX_PIN;
    _delay_ms(1);
    SENS_PORT &= ~SENS_TX_PIN;
    _delay_ms(1);
  



    cnt++;
  
    if( cnt > 25  ) {
			// parse and process data
      memset( buffer, 0, 80 );
      sprintf( buffer, "%u", cap_sense );     
      USART_writeln( buffer );
      cnt = 0;
    } 





		cli();
		data = USART_has_data();
		if( data != NULL ) {
			// parse and process data
      memset( buffer, 0, 80 );
     
			strncpy( buffer, data, 3 );
			cmd = atoi( buffer );

			strncpy( buffer, data+3, 80 );
			value = atoi( buffer );

			switch( cmd ) {
				case 100:
					USART_writeln( "100 Capsense 0.1.1 (c) 2013 Matthias Hannig" );
					break;
			
        // set tone
				case 200:

      
					break;
			}

		}
		sei();
	}	
}


