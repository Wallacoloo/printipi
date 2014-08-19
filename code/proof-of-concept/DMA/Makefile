CC := gcc

all: pi-blaster dma-gpio

pi-blaster: pi-blaster.c 
	$(CC) -I. -Wall -pedantic -o pi-blaster pi-blaster.c
	
dma-gpio: dma-gpio.c
	$(CC) -o dma-gpio dma-gpio.c
	
clean:
	rm -rf pi-blaster dma-gpio
	
.PHONY: clean
