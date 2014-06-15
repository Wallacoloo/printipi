CC=gcc
CPP=g++
CFLAGS=-std=c++0x #Use c++0x for gcc <= 4.6, not c++11
	
all: hrtimermain.o
	$(CPP) hrtimermain.o -o hrtimermain $(CFLAGS) -lrt
	
%.o: %.cpp
	$(CPP) -c -o $@ $< $(CFLAGS)

.PHONY: clean

clean: 
	rm -rf *.o
	rm -rf hrtimermain
