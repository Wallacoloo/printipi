CC=gcc
CPP=g++
#CFLAGS=-std=c++0x -Og -g -flto -fsanitize=address -fno-omit-frame-pointer #Use c++0x for gcc <= 4.6, not c++11
CFLAGS=-std=c++0x -Og -g -flto
#CFLAGS=-std=c++0x -Os -flto
#CFLAGS=-std=c++0x -O3 -flto
LD=ld
LDFLAGS = -flto
OUT=../kossel-firmware
	
all: gparse drivers main.o
	$(CPP) main.o gparse/gparse.o drivers/drivers.o -o $(OUT) $(CFLAGS)
	
%.o: %.cpp
	$(CPP) -c -o $@ $< $(CFLAGS)
	
drivers: drivers/driver.o drivers/kossel.o
	$(LD) -r drivers/driver.o drivers/kossel.o -o drivers/drivers.o $(LDFLAGS)

gparse: gparse/command.o gparse/event.o gparse/scheduler.o gparse/serial.o gparse/state.o
	$(LD) -r gparse/command.o gparse/event.o gparse/scheduler.o gparse/serial.o gparse/state.o -o gparse/gparse.o $(LDFLAGS)
	
.PHONY: clean

clean: 
	#rm -rf gparse/command.o gparse/event.o gparse/serial.o gparse/state.o gparse/gparse.o #alphabetical
	rm -rf gparse/*.o
	rm -rf drivers/*.o
	rm -rf main.o
	rm -rf $(OUT)
