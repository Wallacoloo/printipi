CC=gcc
CXX=g++
#CFLAGS=-std=c++0x -Og -g -flto -fsanitize=address -fno-omit-frame-pointer #Use c++0x for gcc <= 4.6, not c++11
CFLAGS=-std=c++0x -O2 -g -Wall #No -flto for gcc-4.6
#CFLAGS=-std=c++0x -Os -flto
#CFLAGS=-std=c++0x -O3 -flto
LD=ld
LDFLAGS = #-flto
BUILDDIR=../build
OUT=$(BUILDDIR)/kossel-firmware

	
all: gparse drivers $(BUILDDIR)/main.o
	$(CXX) $(BUILDDIR)/main.o $(BUILDDIR)/gparse/gparse.o $(BUILDDIR)/drivers/drivers.o -o $(OUT) $(CFLAGS)
	
$(BUILDDIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) -c -o $@ $< $(CFLAGS)
	
drivers: $(BUILDDIR)/drivers/driver.o $(BUILDDIR)/drivers/kossel.o
	$(LD) -r $^ -o $(BUILDDIR)/drivers/drivers.o $(LDFLAGS)

gparse: $(BUILDDIR)/gparse/command.o $(BUILDDIR)/gparse/event.o $(BUILDDIR)/gparse/scheduler.o $(BUILDDIR)/gparse/serial.o $(BUILDDIR)/gparse/state.o
	$(LD) -r $^ -o $(BUILDDIR)/gparse/gparse.o $(LDFLAGS)
	
.PHONY: clean

clean: 
	#rm -rf gparse/command.o gparse/event.o gparse/serial.o gparse/state.o gparse/gparse.o #alphabetical
	#rm -rf gparse/*.o
	#rm -rf drivers/*.o
	#rm -rf main.o
	#rm -rf $(OUT)
	rm -rf $(BUILDDIR)
	rm -rf $(OUT)
