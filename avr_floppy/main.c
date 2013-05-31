
#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "uart.h"


#define STATUS_PORT PORTC
#define STATUS_DDR  DDRC
#define STATUS_LED  (1<<PC3)

#define FD_PORT PORTC
#define FD_DDR  DDRC

#define FD_DIR  (1<<PC0)
#define FD_STEP (1<<PC1)

#define FD_FORWARD ( FD_PORT &= ~FD_DIR )
#define FD_BACKWARD ( FD_PORT |= FD_DIR )

#define FD_IS_FORWARD (( FD_PORT & FD_DIR ) == 0)
#define FD_IS_BACKWARD (( FD_PORT & FD_DIR ) > 0)

// 80 tracks 
#define FD_POS_MAX 78

uint16_t tones[] = {
  // C, D, E, F, G, A, H, C
  
130,  146, 164, 174, 196, 220, 246, 261,

/*
261,  293, 329, 349, 392, 440, 493, 523,

523, 587, 659, 698, 783, 880, 987, 1046
*/

};


uint8_t cur_tone = 0;


void fd_step() 
{
  // pulse
  FD_PORT &= ~FD_STEP; // pull to ground
  _delay_us(10);
  FD_PORT |= FD_STEP;
  _delay_us(10); 
}

void fd_reset()
{
  uint8_t i;
  FD_BACKWARD;

  for( i = 0; i < FD_POS_MAX; i++ ) {
    fd_step();
    _delay_us(5000);
  }

  FD_FORWARD;
}



/**
 * Step motor control timer interupt
 */
ISR( TIMER1_COMPA_vect ) {
  static uint8_t pos;

  fd_step();
  
  if( FD_IS_FORWARD ) {
    pos++;
  }
  else {
    pos--;
  }

  if( pos >= FD_POS_MAX ) {
    FD_BACKWARD;
  }
  else if( pos == 0 ) {
    FD_FORWARD;
  }

}


void fd_set_tone( uint16_t tone ) {
  // recalculate compare
  TCNT1 = 0;
  OCR1A = (uint16_t) ( (float)F_CPU / ( 1024.0f * (float)tone ))-1;
}

void fd_play()
{
  TIMSK |= (1<<OCIE1A);
}

void fd_stop() 
{
  TIMSK &= ~(1<<OCIE1A);
}


int main()
{
  char  buffer[80];
  char* data;
  uint16_t value;
  uint8_t cmd;

	// wait
	_delay_ms( 100 );
	USART_init();

  FD_DDR |= FD_DIR|FD_STEP;
  STATUS_DDR |= STATUS_LED; 

  fd_reset();
  _delay_ms(100);

  // print welcome 
  USART_writeln( "100 Fl0ppy Music 0.1.1 (c) 2013 Matthias Hannig" );

 
  // setup timer
  TCCR1A = 0;
  TCCR1B = (1<<WGM12)|(1<<CS12)|(1<<CS10); // Clear on match, Prescale: 1024

  sei();


	for(;;) {

/*
       fd_set_tone( tones[cur_tone] );
       fd_play();
       cur_tone++; 

      if( cur_tone >= 8 ) {
        cur_tone = 0;
      }

      _delay_ms( 300 );  

*/

    STATUS_PORT |= STATUS_LED; // led_off	

		cli();
		data = USART_has_data();
		if( data != NULL ) {
			// parse and process data
      memset( buffer, 0, 80 );
     
			strncpy( buffer, data, 3 );
			cmd = atoi( buffer );

			strncpy( buffer, data+4, 80 );
			value = atoi( buffer );

			switch( cmd ) {
				case 100:
					USART_writeln( "100 Fl0ppy Music 0.1.1 (c) 2013 Matthias Hannig" );
					break;
			
        // set tone
				case 200:
          if( value > 0 ) {
            fd_play();
          }
          else {
            fd_stop();
          }

          fd_set_tone( value );
      
					break;
			}

		}

		sei();
	}	

  
}


