CC := gcc
#CFLAGS := -g -std=c99
#CFLAGS := -g -std=c11
CFLAGS := -g -std=gnu99

all: pi-blaster dma-gpio dma-example

pi-blaster: pi-blaster.c 
	$(CC) -I. -Wall -pedantic -o pi-blaster pi-blaster.c
	
dma-gpio: dma-gpio.c
	$(CC) $(CFLAGS) -o dma-gpio dma-gpio.c
	
dma-example: dma-example.c
	$(CC) $(CFLAGS) -o dma-example dma-example.c
	
clean:
	rm -rf pi-blaster dma-gpio
	
.PHONY: clean
