
#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <math.h>

#include "uart.h"

uint16_t cap_sense;

#define STATUS_PORT PORTB      
#define STATUS_DDR  DDRB       
#define STATUS_LED  (1<<PB0)   


#define LAMPS_DDR   DDRB
#define LAMPS_PORT  PORTB
#define LAMP_0      (1<<PB0)
#define LAMP_1      (1<<PB1)
#define LAMP_2      (1<<PB2)
#define LAMP_3      (1<<PB3)
#define LAMP_4      (1<<PB4)
#define LAMP_5      (1<<PB5)
#define LAMP_6      (1<<PB6)



volatile uint8_t cnt_lamps;
volatile uint8_t cmp_lamps[6];


ISR (TIMER0_OVF_vect)
{
  int i;
  cnt_lamps++; 

  for( i = 0; i < 6; i++ ) {
    if( cnt_lamps > cmp_lamps[i] ) {
      LAMPS_PORT &= ~(1<<i);
    }    
    else {
      LAMPS_PORT |= (1<<i);
    }
  }

  // Preload timer
  TCNT0 = 170;
}



int main()
{
	USART_init();

  // Lamp 0 output.
  LAMPS_DDR |= LAMP_0|LAMP_1|LAMP_2|LAMP_3|LAMP_4|LAMP_5;


  // Setup soft pwm timer  
  TCCR0 = (1<<CS01); //|(1<<CS00); 
  TIMSK |= (1<<TOIE0); // allow overflow interrupt

  sei();

  uint8_t t = 0;


  int i = 0;
  for( i = 0; i < 6; i++ ) {
    cmp_lamps[i] = 0;
  }  

  uint16_t value = 0;
  uint8_t dir = 1;

	for(;;) {
    t++;
    STATUS_PORT ^= STATUS_LED;

    if( dir ) {
      value++;
    }
    else {
      value--;
    }

    cmp_lamps[0] = (uint8_t) (((sin(((float)t/255.0f)*6.28f))+1.0f) * 127.0f);
    cmp_lamps[1] = (uint8_t) (((sin(((float)(t+25)/255.0f)*6.28f))+1.0f) * 127.0f);
    cmp_lamps[2] = (uint8_t) (((sin(((float)(t+80)/255.0f)*6.28f))+1.0f) * 127.0f);
    cmp_lamps[3] = (uint8_t) (((sin(((float)(t+180)/255.0f)*6.28f))+1.0f) * 127.0f);
    cmp_lamps[4] = (uint8_t) (((sin(((float)(t+220)/255.0f)*6.28f))+1.0f) * 127.0f);
    cmp_lamps[5] = (uint8_t) (((sin(((float)(t+60)/255.0f)*6.28f))+1.0f) * 127.0f);


    if( value == 0 ) dir = 1;
    if( value > 200 ) dir = 0;


    _delay_ms( 5 );
	}	
}


