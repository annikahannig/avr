
#ifndef F_CPU
#define F_CPU 4000000UL
#endif


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "uart.h"




#define LED_PORT  PORTD
#define LED_DDR   DDRD 

#define LED_STR   (1<<PD7)
#define LED_DATA  (1<<PD6)
#define LED_CLK   (1<<PD5)
#define LED_OE    (1<<PD4)


void leds_init()
{
    LED_DDR |= LED_STR | LED_DATA | LED_CLK | LED_OE;
}

void leds_write( char c )
{
    int i;
    char v;

    LED_PORT |= LED_OE; // enable output
    LED_PORT &= ~LED_STR; // strobe low

    // bitwise push
    for( i = 0; i<8; i++ ) {
        // clock low
        LED_PORT &= ~LED_CLK;

        v = 1&(c>>i);

        // set value (inverted; since the led
        if( v ) {
            LED_PORT &= ~LED_DATA;// 0
        }
        else {
            LED_PORT |= LED_DATA; // 1
        }

        _delay_us( 1 ); // setup time
        LED_PORT |= LED_CLK; // clock high
        _delay_us( 1 ); // hold time / clock pulse width
    }

    // lower clock
    LED_PORT &= ~LED_CLK;

    // push to output register
    LED_PORT |= LED_STR;
    _delay_us( 1 );
    LED_PORT &= ~LED_STR;
}



inline uint8_t debounce(volatile uint8_t *port, uint8_t pin)
{
    if ( *port & (1 << pin) )
    {
        /* Pin wurde auf Masse gezogen, 100ms warten   */
        _delay_ms(20);   // Maximalwert des Parameters an _delay_ms 
        _delay_ms(20);   // beachten, vgl. Dokumentation der avr-libc
        if ( *port & (1 << pin) )
        {
            /* Anwender Zeit zum Loslassen des Tasters geben */
            _delay_ms(20);
            _delay_ms(20); 
            return 1;
        }
    }
    return 0;
}








#define MOTOR (1<<PB1)
#define PWM_DDR DDRB

#define INPUT_PORT PORTC
#define INPUT_DDR  DDRC

#define T_UP       PC5
#define T_DN       PC4


int main()
{
	char* data;	

	// initialize output leds
    leds_init();
    leds_write( 0b11111111 );


	// initialize input 
	INPUT_DDR  &= ~( ( 1<<T_UP ) |  (1<<T_DN) ); // input
	INPUT_PORT |=  (1<<T_UP) | (1<<T_DN); // pullups

	

	// initialize PWM
	PWM_DDR |= MOTOR;
	
	// non inverting pwm COM1A1:1 COM1A0:0
    // phase correct 9 bit pwm WGM11:1 (WGM_other: 0)
	TCCR1A = (1<<COM1A1) | (1<<WGM11);
	TCCR1B = (1<<WGM13)  | (1<<WGM12) | (1<<CS10) | (1<<CS11);
	
	// ICR1 = 255;
	ICR1 = 255;

	// PWM off
	OCR1A = 0;

	uint8_t val = 0;

	// initialize USART
	USART_init();

	// initialize UART BT device
	_delay_ms( 500 );
	_delay_ms( 500 );
	_delay_ms( 500 );
	_delay_ms( 500 );
	USART_writeln( "AT+NAMEbtpt" );
	leds_write( 0b10101010 );
	_delay_ms( 500 );
	USART_writeln( "AT+PIN1337" );
	_delay_ms( 500 );
	leds_write( 0b11110000 );
	_delay_ms( 500 );


	sei();

//	USART_writeln( "PController version 0.0.1" );
//	USART_writeln( "(c) 2011 Matthias Hannig" );


	char buffer[80];	

	int cmd;
	int value;

	uint8_t change = 0;	

	for(;;) {


		if( debounce( &PINC, T_UP ) ) {
			val++;
			change = 1;
		}
		else if( debounce( &PINC, T_DN ) ) {
			val--;
			change = 1;
		}
    	
		/*
		if( val > 126 ) {
			val = 126;
		}
		else if( val < 0 ) {
			val = 0;
		}
		*/

		if( change ) {
			sprintf( buffer, "401 %d", val );
			USART_writeln( buffer );
			change = 0;
		}


		// serial port processing 
		/*
		 * The pprotocol
		 * -------------
		 *
		 * 100 Software Version & Copyright
		 * 110 Connected machines 
		 * 400 Set Current Value Motor A
		 * 401 GET motor value
		 */
		cli();
		if( ( data = USART_has_data() ) != NULL ) {
			// parse and process data
			strncpy( buffer, data, 3 ); 
			cmd = atoi( buffer );
				
			strncpy( buffer, data+3, 80 );
			value = atoi( buffer );
			
			switch( cmd ) {
				case 100:
					USART_writeln( "100 Panty Controller Version 0.0.1 (c) 2011 Matthias Hannig" );
					break;

				case 110:
					USART_writeln( "110 1" );
					break;

				case 400:
					val = value;	
					sprintf( buffer, "401 %d", val );
					USART_writeln( buffer );
					break;

				case 401:
					sprintf( buffer, "401 %d", val );
					USART_writeln( buffer );
					break;

			}
		
		}
		sei();
	
		leds_write( val );
		OCR1A = val;
	}
	
}


