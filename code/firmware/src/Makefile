CC=gcc
CPP=g++
CFLAGS=-std=c++11

%.o: %.cpp
	$(CPP) -c -o $@ $< $(CFLAGS)

all: main.o command.o
	$(CPP) main.o command.o -o ../kossel-firmware $(CFLAGS)
	
.PHONY: clean

clean: main.o
	rm main.o
