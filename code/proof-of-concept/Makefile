CXX=g++
CFLAGS=-std=c++0x -O2 #Use c++0x for gcc <= 4.6, not c++11
	
all: hrtimermain gpiotest stdchronomain
hrtimermain: hrtimermain.o
	$(CXX) hrtimermain.o -o hrtimermain $(CFLAGS) -lrt
gpiotest: gpiotest.o
	$(CXX) gpiotest.o -o gpiotest $(CFLAGS) -lwiringPi -pthread
stdchronomain: stdchronomain.o
	$(CXX) stdchronomain.o -o stdchronomain $(CFLAGS)
	
%.o: %.cpp
	$(CXX) -c -o $@ $< $(CFLAGS)

.PHONY: clean

clean: 
	rm -rf *.o
	rm -rf hrtimermain gpiotest stdchronomain
