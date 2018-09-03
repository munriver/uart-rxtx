MCU = atmega8

F_CPU = 8000000 #Processor freq

FORMAT = ihex

TARGETS = rx.c tx.c

CSTANDARD = -std=gnu99

CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
AVRDUDE = avrdude
PROGRAMMER = usbasp
REMOVE = rm -f

obj := $(TARGETS:.c=.o)
elf := $(TARGETS:.c=.elf)
hex := $(TARGETS:.c=.hex)


all: $(elf) build 

build: $(hex) 

%.elf: %.o
	$(CC) -g -mmcu=$(MCU) -o $@ $<

%.o: %.c 
	$(CC) -g -Os -mmcu=$(MCU) -c $<  

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O $(FORMAT) $< $@ 
	@echo "HEX size"
	@$(SIZE) --target=$(FORMAT) $@

programtx: tx.hex 
	$(AVRDUDE) -p $(MCU) -c $(PROGRAMMER) -U flash:w:tx.hex
programrx: rx.hex 
	$(AVRDUDE) -p $(MCU) -c $(PROGRAMMER) -U flash:w:rx.hex

clean:
	$(REMOVE) *.hex
	$(REMOVE) *.elf
	$(REMOVE) *.o
	
