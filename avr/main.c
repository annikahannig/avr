
#ifndef F_CPU
#define F_CPU 4000000UL
#endif

#define BAUD 9600UL

#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)   // clever runden
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))     // Reale Baudrate
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD) // Fehler in Promille, 1000 = kein Fehler.
 
#if ((BAUD_ERROR<990) || (BAUD_ERROR>1010))
#error Error in baudrate > 0.1
#endif 

#define UART_BUFFER_SIZE 80

#define UART_ECHO 0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#include "lcd.h"

volatile uint8_t uart_string_ready = 0;
volatile uint8_t uart_string_length = 0;
volatile char    uart_string_buffer[UART_BUFFER_SIZE] = "";

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



/**
 * New implementation of LCD display 
 * driver (AV2020) 
 */

/*
#define mLCD_PORT PORTB
#define mLCD_DDR  DDRB

#define mLCD_EN   (1<<PB5)
#define mLCD_RS   (1<<PB4)

#define LCD_PORT_MASK ((1<<PB5)|(1<<PB4)|(1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0))
#define LCD_DATA_MASK ((1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0))

#define LCD_ENABLE() \
	LCD_PORT |= mLCD_EN; \
	_delay_ms( 20 ); \
	LCD_PORT &= ~mLCD_EN;

#define LCD_COMMAND(cmd) \
	LCD_PORT &= ~mLCD_RS; \
	lcd_send_byte(cmd);


void lcd_init()
{
	LCD_DDR |= LCD_PORT_MASK; // LCD ports as output

	_delay_ms( 40 ); // wait more than 30 ms	
	LCD_PORT &= ~LCD_PORT_MASK; // OFF
	
	// init 
	LCD_PORT |= 0b00000011;
	
	LCD_ENABLE();
	_delay_ms(5);
	LCD_ENABLE();
	_delay_ms(1);
	LCD_ENABLE();
	_delay_ms(1);


	LCD_PORT &= 0xF0;
	LCD_PORT |= 0x02;
	lcd_enable();
	_delay_ms(3);

	// 4Bit / 2 Zeilen / 5x20
	// lcd_command(0x0E); // 0E  4x20 Zeile
	LCD_COMMAND(0b00101100); // 0E  4x20 Zeile

	// Display ein / Cursor aus / kein Blinken
	LCD_COMMAND(0x0C);

	// inkrement / kein Scrollen
	LCD_COMMAND(0x06);

	lcd_clear();

}

*/


/**
 * UART receive interrupt
 */
ISR( USART_RXC_vect )
{
	static uint8_t count;
	unsigned char c = UDR;

	if( uart_string_ready == 0 ) {
		if( c != '\n' && c != '\r' &&
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

/* Zeichen empfangen */
uint8_t uart_getc(void)
{
    while (!(UCSRA & (1<<RXC)))   // warten bis Zeichen verfuegbar
        ;
    return UDR;                   // Zeichen aus UDR an Aufrufer zurueckgeben
}
 
void uart_gets( char* Buffer, uint8_t MaxLen )
{
  uint8_t NextChar;
  uint8_t StringLen = 0;
 
  NextChar = uart_getc();         // Warte auf und empfange das nächste Zeichen
 
                                  // Sammle solange Zeichen, bis:
                                  // * entweder das String Ende Zeichen kam
                                  // * oder das aufnehmende Array voll ist
  while( NextChar != '\n' && StringLen < MaxLen - 1 ) {
    *Buffer++ = NextChar;
    StringLen++;
    NextChar = uart_getc();
  }
 
                                  // Noch ein '\0' anhängen um einen Standard
                                  // C-String daraus zu machen
  *Buffer = '\0';
}


void PWM_init()
{	
	DDRB = ( 1 << PB1 );
	// invertiertest pwm (com1a1 com1a0)
	// wgm11 9 bit pwm
	TCCR1A = (1<<COM1A1) | (1<<COM1A0) | (1<<WGM11);
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
	
	ICR1 = 0x00ff;
}


char anim[] = {
	0b11001100, 
	0b00110011,
	0b11001100, 
	0b00110011,
	0b11001100, 
	0b00110011,
	0b11001100,
	0b00110011,
	0b10011001,
	0b11001100,
	0b01100110,
	0b00110011,
	0b01010101,
	0b00110011,
	0b00110011,
	0b11001100,
	0b00110011,
	0b10011001,
	0b11001100,
	0b01100110,
	0b00110011,
	0b01010101,
	0b11001100,
	0b00110011,
	0b10011001,
	0b11001100,
	0b01100110,
	0b00110011,
	0b01010101,
	0b10101010,
	0b01010101,
	0b01010100,
	0b01010000,
	0b01000000,
	0b00100000,
	0b00010000,
	0b00001000,
	0b00000100,
	0b00000010,
	0b00000001,
	0b10000000,
	0b01000000,
	0b00100000,
	0b00010000,
	0b00001000,
	0b00000100,
	0b00000010,
	0b00000001,
	0b10000000,
	0b01000000,
	0b00100000,
	0b00010000,
	0b00001000,
	0b00000100,
	0b00000010,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001001,
	0b00010010,	
	0b00100100,	
	0b01001000,
	0b10010010,
	0b00100100,
	0b01001001,
	0b10010010,
	0b00100100,
	0b00010010,	
	0b00100100,	
	0b01001000,
	0b10010010,
	0b10100100,
	0b11010100,
	0
};


int main()
{
	int count = 0;
	int alen  = 0;
	char msg[80];
//	char buf[80];
	uint16_t led_val = 0;

	alen = strlen( anim );

	lcd_init();
	lcd_clear();
	lcd_set_cursor(0,1);
	lcd_string( "Hallo wellt" );

	USART_init();

//	PWM_init();

	leds_init();
	leds_write( 0b11001100 );

	DDRC = 0xff;


	// print startup stuff

//	USART_writeln( " L33t thing version 0.1 " );
//	USART_writeln( " (c) 2011 Matthias Hannig" );
//	USART_puts( "\nrdy> " ); 

	// initialize UART BT device
	_delay_ms( 500 );
	_delay_ms( 500 );
	_delay_ms( 500 );
	_delay_ms( 500 );
	USART_writeln( "AT+NAMEbtpant" );
	leds_write( 0b10101010 );
	_delay_ms( 500 );
	USART_writeln( "AT+PIN1337" );
	_delay_ms( 500 );
	leds_write( 0b11110000 );
	_delay_ms( 500 );


	sei(); // enable interrupts

//	OCR1A = 0xff;

	while(1) {

//		PORTC = 0xff;

		
/*
		uart_gets( &buf, 80 );
	
		memset( msg, 0, 80 );
		sprintf( msg, "Received: %s", buf );
		
		USART_writeln( msg );
*/

		if( uart_string_ready ) {
			cli();
			memset( msg, 0, 80 );
			sprintf( msg, "Received: [%s]", uart_string_buffer );
//			USART_writeln( msg );			
			sscanf( (const char*) uart_string_buffer, "%d", &led_val );
			// yay
			// mach irgendwas mit dem received string..
	
/*			if( strcmp( (const char*) uart_string_buffer, "on" ) == 0 ) {
				PORTC = 0x00;
			}
			else {
/				OCR1A = led_val;
				PORTC = 0xff;
			}
*/
			USART_writeln( "OK" );
			uart_string_ready = 0;
			
			USART_puts( "rdy> " );
			sei();
		}

		_delay_ms(90);

		leds_write( anim[count%alen] );

		count++;
	}

	return 0;
}


