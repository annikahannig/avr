
#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <math.h>
#include "mth.h"

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
volatile uint8_t cmp_lamps[7];

volatile float   int_lamps[7];
volatile float   set_lamps[7];

volatile float   t_lamps[7];
volatile uint8_t x_lamps[7];


inline void set_intensity( uint8_t lamp, float i )
{
  set_lamps[lamp] = i;
  t_lamps[lamp]   = 0.0f;
  x_lamps[lamp]  |= (1<<lamp); 
}

inline void set_lamp_cmp( uint8_t lamp )
{
  cmp_lamps[lamp] = (uint8_t) (int_lamps[lamp] * 255.0f);
}


// lamp kernels
void k_pulse( uint8_t lamp )
{
  if( (x_lamps[lamp] & (1<<lamp)) == 0 ) {
    return;
  }
  
  if( t_lamps[lamp] > 1 ) {
    x_lamps[lamp] &= ~(1<<lamp);
    return;
  }
  
  int_lamps[lamp] = m_pulse_to( 10, set_lamps[lamp], t_lamps[lamp] );  
  t_lamps[lamp] += 0.04;  
}


// Candle kernel
void k_candle( uint8_t lamp) 
{
  if( (x_lamps[lamp] & (1<<lamp)) == 0 ) {
    return;
  }

  if( t_lamps[lamp] > 4 ) {
    t_lamps[lamp] = 0;
  }
  
  if( t_lamps[lamp] == 0 ) {
    t_lamps[lamp] = (float)lamp*0.2;
  }

  float x = t_lamps[lamp];
  float v = 0.5 + 0.5*sin(x*4)*sin(x*x*0.5)*cos(10*x)*cos(x*x*0.5)*sin(x*x*20);
  
  int_lamps[lamp] = v;
  t_lamps[lamp] += 0.01;
}


ISR (TIMER0_OVF_vect)
{
  int i;
  cnt_lamps++; 

  for( i = 0; i < 7; i++ ) {
    if( cnt_lamps >= cmp_lamps[i] ) {
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
  uint8_t cnt = 0;

  char buf[23];


  int i = 0;
  for( i = 0; i < 7; i++ ) {
    cmp_lamps[i] = 0;
    int_lamps[i] = 0.0f;
    set_lamps[i] = 0.0f;
    set_intensity( i, 1.0f );
  }  


  // Render 
	for(;;) {
    /*
    // trigger pulses 
    if( t == 1 ) {
     //  set_intensity( 0, 0.005f );
      USART_writeln( "i 0: 0.1" );
    }  

    if( t == 42 ) {
      set_intensity( 1, 0.005f );
      USART_writeln( "i 1: 0.1" );
    }  
    
    if( t == 84 ) {
      set_intensity( 2, 0.005f );
      USART_writeln( "i 2: 0.1" );
    }  

    if( t == 126 ) {
      set_intensity( 3, 0.005f );
      USART_writeln( "i 3: 0.1" );
    }  

    if( t == 168 ) {
      set_intensity( 4, 0.05f );
      USART_writeln( "i 3: 0.1" );
    }  

    if( t == 210 ) {
      set_intensity( 5, 0.05f );
      USART_writeln( "i 3: 0.1" );
    }  

    */

    for( i = 0; i < 6; i++) {
      k_candle(i);
    }
    

    // k_candle( 0 );
  
    // update lamp models
    for( i = 1; i < 7; i++ ) {
      // k_pulse( i );
    }

    // update lamp cmp
    for( i = 0; i < 7; i++ ) {
      set_lamp_cmp(i);
    }

    // debug
    
    t++;
    _delay_ms( 5 );
	}	
}


