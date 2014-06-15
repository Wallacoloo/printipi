CC=gcc
CPP=g++
CFLAGS=-std=c++0x #Use c++0x for gcc <= 4.6, not c++11
LD=ld
OUT=../kossel-firmware
	
all: gparse driver.o main.o
	$(CPP) main.o gparse/gparse.o driver.o -o $(OUT) $(CFLAGS)
	
%.o: %.cpp
	$(CPP) -c -o $@ $< $(CFLAGS)

gparse: gparse/command.o gparse/serial.o gparse/state.o
	ld -r gparse/command.o gparse/serial.o gparse/state.o -o gparse/gparse.o
	
.PHONY: clean

clean: 
	rm -rf gparse/command.o gparse/serial.o gparse/state.o gparse/gparse.o #alphabetical
	rm -rf driver.o main.o
	rm -rf $(OUT)
