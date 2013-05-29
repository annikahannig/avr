
CC=avr-gcc

CCFLAGS=-mmcu=atmega8 -I. -gdwarf-2 -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall  -Wundef  -Wundef -MD -MP

all:
	$(CC) $(CCFLAGS) -c -o uart.o uart.c
	$(CC) $(CCFLAGS) main.c uart.o -o fl0ppy.elf 
	avr-objcopy  -O ihex -R .eeprom fl0ppy.elf fl0ppy.hex

flash:
	sudo avrdude -c avr910 -p m8 -P /dev/ttyUSB1 -e -U flash:w:fl0ppy.hex

fuse_xtal:
	sudo avrdude -c avr910 -p m8 -P /dev/ttyUSB0 -U lfuse:w:0xff:m -U hfuse:w:0xd9:m 
