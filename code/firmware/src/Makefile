CC=gcc
CPP=g++
CFLAGS=-std=c++0x #Use c++0x for gcc <= 4.6, not c++11
LD=ld
OUT=../kossel-firmware
	
all: main.o gparse
	$(CPP) main.o gparse/gparse.o -o $(OUT) $(CFLAGS)
	
%.o: %.cpp
	$(CPP) -c -o $@ $< $(CFLAGS)

gparse: gparse/command.o gparse/serial.o
	ld -r gparse/command.o gparse/serial.o -o gparse/gparse.o
	
.PHONY: clean

clean: 
	rm -rf main.o
	rm -rf gparse/command.o
	rm -rf gparse/serial.o
	rm -rf gparse/gparse.o
	rm -rf $(OUT)
