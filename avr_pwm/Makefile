
CC=avr-gcc

CCFLAGS=-mmcu=atmega8 -I. -gdwarf-2 -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall  -Wundef  -Wundef -MD -MP

all:
	$(CC) $(CCFLAGS) -c -o lcd.o lcd.c 
	$(CC) $(CCFLAGS) -c -o uart.o uart.c
	$(CC) $(CCFLAGS) main.c lcd.o uart.o -o test.elf 
	avr-objcopy  -O ihex -R .eeprom test.elf test.hex


flash:
	sudo avrdude -c avr910 -p m8 -P /dev/ttyUSB0 -e -U flash:w:test.hex

fuse_xtal:
	sudo avrdude -c avr910 -p m8 -P /dev/ttyUSB1 -U lfuse:w:0xff:m -U hfuse:w:0xd9:m 
