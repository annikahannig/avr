/*
 * Simple_AVR_MIDI.c
 *
 *  Created on: 30.10.2010
 *      Author: Felix Wienker
 */


#include <avr/io.h>
#include <stdlib.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

void waitms(uint16_t ms)
{
	for(; ms>0; ms--)
	{
		uint16_t __c = 4000;
		__asm__ volatile (
			"1: sbiw %0,1" "\n\t"
			"brne 1b"
			: "=w" (__c)
			: "0" (__c)
		);
	}
}

void init_USART(void)
{
	UCSRB |= (1<<TXEN);					//UART TX (Transmit enable)
	UCSRC |= (1<<URSEL)|(3<<UCSZ0);		//Mode Async 8N1 (8 Databits, No Parity, 1 Stopbit)

										//UBRR defines baudrate
										//Baudrate for MIDI is 31250
										//UBRR = (Clock-frequency / (Baudrate * 16)) - 1
										//Clock-frequency = 16000000 Hz => UBRR = 31
	UBRRH = 0;							//Highbyte is 0
	UBRRL = 31;							//Lowbyte ist 31 (decimal)
}

void sendchar(unsigned char c)
{
	while(!(UCSRA & (1<<UDRE)))
	{
	}
	UDR = c;
}


void play_C_Chord(void){

		//////////////////////
		// Play C Major Chord
		//////////////////////

		sendchar(0x90); //Note ON
		sendchar(0x3C); //Note: C3
		sendchar(0x7F); //Velocity: 127(Max)

		sendchar(0x40); //Note: E3
		sendchar(0x7F); //Velocity: 127(Max)

		sendchar(0x43); //Note: G3
		sendchar(0x7F); //Velocity: 127(Max)


		////////////////////
		// Wait 1 second
		////////////////////

		waitms(1000);


		//////////////////////
		// Stop playing Chord
		//////////////////////
		sendchar(0x80); //Note OFF
		sendchar(0x3C); //Note: C3
		sendchar(0x00); //Velocity: 0

		sendchar(0x40); //Note: E3
		sendchar(0x00); //Velocity: 0

		sendchar(0x43); //Note: G3
		sendchar(0x00); //Velocity: 0
}

int main(int argc, char **argv) {
	init_USART();


  for(;;) {

	  play_C_Chord();
    
    waitms( 1400 );
  }

}




