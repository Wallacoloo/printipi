CC := gcc
#CFLAGS := -g -std=c99
#CFLAGS := -g -std=c11
CFLAGS := -g -std=gnu99

all: dma-gpio dma-example

pi-blaster: pi-blaster.c 
	$(CC) -I. -Wall -pedantic -o pi-blaster pi-blaster.c
	
dma-gpio: dma-gpio.c
	$(CC) $(CFLAGS) -O2 -o dma-gpio dma-gpio.c -lrt 
	
dma-example: dma-example.c
	$(CC) $(CFLAGS) -o dma-example dma-example.c
	
clean:
	rm -rf pi-blaster dma-gpio
	
.PHONY: clean
