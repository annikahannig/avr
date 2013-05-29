#ifndef LCD_H
#define LCD_H

void lcd_data(unsigned char byte);
void lcd_string(char *data);

void lcd_send_byte(unsigned char byte);
void lcd_command(unsigned char byte);

void lcd_enable(void);
void lcd_init(void);
void lcd_home(void);
void lcd_clear(void);

void lcd_set_cursor(uint8_t x, uint8_t y);

// Hier die verwendete Taktfrequenz in Hz eintragen, wichtig!

#ifndef F_CPU
#define F_CPU 4000000UL
#endif

// LCD Befehle

#define CLEAR_DISPLAY 0x01
#define CURSOR_HOME   0x02

// Pinbelegung für das LCD, an verwendete Pins anpassen
#define LCD_PORT      PORTB
#define LCD_DDR       DDRB
#define LCD_RS        PB4
#define LCD_EN        PB5
// DB4 bis DB7 des LCD sind mit PD0 bis PD3 des AVR verbunden


#endif
