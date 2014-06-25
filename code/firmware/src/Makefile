CC=gcc
CPP=g++
CFLAGS=-std=c++0x #Use c++0x for gcc <= 4.6, not c++11
LD=ld
OUT=../kossel-firmware
	
all: gparse drivers main.o
	$(CPP) main.o gparse/gparse.o drivers/drivers.o -o $(OUT) $(CFLAGS)
	
%.o: %.cpp
	$(CPP) -c -o $@ $< $(CFLAGS)
	
drivers: drivers/driver.o drivers/kossel.o
	ld -r drivers/driver.o drivers/kossel.o -o drivers/drivers.o

gparse: gparse/command.o gparse/event.o gparse/serial.o gparse/state.o
	ld -r gparse/command.o gparse/event.o gparse/serial.o gparse/state.o -o gparse/gparse.o
	
.PHONY: clean

clean: 
	#rm -rf gparse/command.o gparse/event.o gparse/serial.o gparse/state.o gparse/gparse.o #alphabetical
	rm gparse/*.o
	rm drivers/*.o
	rm -rf main.o
	rm -rf $(OUT)
