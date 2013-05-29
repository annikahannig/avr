
#include <avr/io.h>
#include "lcd.h"
#include <util/delay.h>

#include <avr/interrupt.h>




void lcd_enable(void)
{
   LCD_PORT |= (1<<LCD_EN);
    _delay_us(20);  // respect lcd timings
   LCD_PORT &= ~(1<<LCD_EN); // turn off
}


void lcd_send_byte( unsigned char byte )
{

        unsigned char byte_hi = byte;

        byte_hi = byte_hi >> 4;    // upper nibble
        byte_hi = byte_hi & 0x0F;  // maskieren

        LCD_PORT &= 0xF0;
        LCD_PORT |= byte_hi;   // setzen


        lcd_enable(); // set enable bit

        byte = byte & 0x0F; // unteres nibble

        LCD_PORT &= 0xF0;
        LCD_PORT |= byte;  // setzen

        lcd_enable();

        _delay_us(75);
}


// sendet ein Datenbyte an das LCD
void lcd_data(unsigned char byte)
{
	LCD_PORT |= (1<<LCD_RS);  // RS auf 1 setzen
	lcd_send_byte( byte );
}


void lcd_command(unsigned char byte)
{
	LCD_PORT &= ~(1<<LCD_RS);  // RS LOW!!
	lcd_send_byte( byte );
}



void lcd_clear(void)
{
   lcd_command(CLEAR_DISPLAY);
   _delay_ms(15);
}

// Sendet den Befehl: Cursor Home

void lcd_home(void)
{
   lcd_command(CURSOR_HOME);
   _delay_ms(15);
}

void lcd_set_cursor(uint8_t x, uint8_t y)
{
  uint8_t tmp;

  switch (y) {
    case 0: tmp=0x80+0x00+x; break;    // 1. Zeile
    case 1: tmp=0x80+0x40+x; break;    // 2. Zeile
    case 2: tmp=0x80+0x10+x; break;    // 3. Zeile
    case 3: tmp=0x80+0x50+x; break;    // 4. Zeile
    default: return;                   // für den Fall einer falschen Zeile
  }
  lcd_command(tmp);
}

// Schreibt einen String auf das LCD

void lcd_string(char *data)
{
    while(*data) {
        lcd_data(*data);
        data++;
    }
}



void lcd_init(void)
{
	LCD_DDR = LCD_DDR | 0x0F | (1<<LCD_RS) | (1<<LCD_EN);   // Port auf Ausgang schalten

	_delay_ms(42); // after powerup
	LCD_PORT &= 0xF0;
	LCD_PORT |= 0x03;
	LCD_PORT &= ~(1<<LCD_RS);      // RS auf 0

	lcd_enable();
	_delay_ms(5);
	lcd_enable();
	_delay_ms(1);
	lcd_enable();
	_delay_ms(1);

	// 4 Bit Modus aktivieren
	LCD_PORT &= 0xF0;
	LCD_PORT |= 0x02;
	lcd_enable();
	_delay_ms(3);

	// 4Bit / 2 Zeilen / 5x20
	// lcd_command(0x0E); // 0E  4x20 Zeile
	lcd_command(0b00101100); // 0E  4x20 Zeile

	// Display ein / Cursor aus / kein Blinken
	lcd_command(0x0C);

	// inkrement / kein Scrollen
	lcd_command(0x06);

	lcd_clear();
}

