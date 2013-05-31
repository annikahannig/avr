
CC=avr-gcc

CCFLAGS=-mmcu=atmega8 -I. -gdwarf-2 -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall  -Wundef  -Wundef -MD -MP

PROGRAM=avr_lpwm

PART=m8

# OS Detection
UNAME=$(shell uname)
ifeq ($(UNAME), Darwin) 
  PROG_PRT=tty.SLAB_USBtoUART
else
  PROG_PRT=ttyUSB1
endif

all:
	$(CC) $(CCFLAGS) -c -o uart.o uart.c
	$(CC) $(CCFLAGS) main.c uart.o -o $(PROGRAM).elf 
	avr-objcopy  -O ihex -R .eeprom $(PROGRAM).elf $(PROGRAM).hex

flash:
	sudo avrdude -c avr910 -p $(PART) -P /dev/$(PROG_PRT) -e -U flash:w:$(PROGRAM).hex

fuse_xtal:
	sudo avrdude -c avr910 -p $(PART) -P /dev/$(PROG_PRT) -U lfuse:w:0xff:m -U hfuse:w:0xd9:m 

fuse_osc:
	sudo avrdude -c avr910 -p $(PART) -P /dev/$(PROG_PRT) -U lfuse:w:0xe0:m -U hfuse:w:0xd9:m 
 
