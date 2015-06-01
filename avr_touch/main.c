
#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <math.h>

#include "uart.h"


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


#define SENS_IN   PINC
#define SENS_PORT PORTC
#define SENS_DDR  DDRC

#define SENS_0_TX    (1<<PC5)
#define SENS_0_RX    (1<<PC4)

#define SENS_1_TX    (1<<PC3)
#define SENS_1_RX    (1<<PC2)

#define SENS_2_TX    (1<<PC1)
#define SENS_2_RX    (1<<PC0)


volatile uint16_t cnt_sens[3];


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
  LAMPS_DDR |= LAMP_0|LAMP_1|LAMP_2|LAMP_3|LAMP_4;

  // Reset sensor counter
  cnt_sens[0] = cnt_sens[1] = cnt_sens[2] = 0;

  // pull down caps
  SENS_DDR |= SENS_0_TX|SENS_0_RX|
              SENS_1_TX|SENS_1_RX|
              SENS_2_TX|SENS_2_RX;

  SENS_PORT = 0;

  SENS_DDR = 0; // All Pins as Input (Z State)


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

/*

   
    // Push charge
    SENS_PORT |= SENS_0_TX|SENS_1_TX|SENS_2_TX;
    _delay_us(10);
    SENS_PORT &= ~(SENS_0_TX);
    SENS_PORT &= ~(SENS_1_TX);
    SENS_PORT &= ~(SENS_2_TX);

    // Pull rx to ground
    SENS_DDR  |= SENS_0_RX|SENS_1_RX|SENS_2_RX;
    _delay_us(100);

    // set RX to Z
    SENS_DDR &= ~(SENS_0_RX);
    SENS_DDR &= ~(SENS_1_RX);
    SENS_DDR &= ~(SENS_2_RX);

    // increment cycle counter
    cnt_sens[0]++;
    cnt_sens[1]++;
    cnt_sens[2]++;

    // check
    if( SENS_IN & SENS_0_TX ) {

      // how many cycles?
      if( cnt_sens[0] < 50 ) {
        // we are touched
        cmp_lamps[4] = 255;
      }
      else {
        cmp_lamps[4] = 0;
      }

      cnt_sens[0] = 0;
    
      // pull to ground
      SENS_DDR |= SENS_0_RX|SENS_0_TX;
      SENS_PORT &= ~SENS_0_TX;
      SENS_PORT &= ~SENS_0_RX;
      _delay_us(500);
      
      SENS_DDR &= ~SENS_0_TX;
      SENS_DDR &= ~SENS_0_RX;
    }
    

*/


    cmp_lamps[0] = (uint8_t) (((sin(((float)t/255.0f)*6.28f))+1.0f) * 127.0f);
    cmp_lamps[1] = (uint8_t) (((sin(((float)(t+25)/255.0f)*6.28f))+1.0f) * 127.0f);
    cmp_lamps[2] = (uint8_t) (((sin(((float)(t+80)/255.0f)*6.28f))+1.0f) * 127.0f);
    cmp_lamps[3] = (uint8_t) (((sin(((float)(t+180)/255.0f)*6.28f))+1.0f) * 127.0f);
    cmp_lamps[4] = (uint8_t) (((sin(((float)(t+220)/255.0f)*6.28f))+1.0f) * 127.0f);

//    _delay_ms( 5 );
	}	
}


