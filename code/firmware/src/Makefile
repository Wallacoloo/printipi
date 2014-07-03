CXX=g++
CFLAGS=-std=c++0x -mtune=native
#CFLAGS=-std=c++0x -Og -g -flto -fsanitize=address -fno-omit-frame-pointer #Use c++0x for gcc <= 4.6, not c++11
LD=ld
LDFLAGS=
#FLTOFLAG=-flto #Enable link-time optimization
LTOFLAG= #Disable link-time optimization, as it often leads to link-time errors for gcc < 4.8
BUILDROOT=../build
DEBUGDIR=$(BUILDROOT)/debug
RELEASEDIR=$(BUILDROOT)/release
NAME=kossel-firmware

all: debug

#Configure debug/release settings:
debug: CFLAGS+=-O2 -g -Wall
debug: $(DEBUGDIR)/$(NAME)
release: CFLAGS+= -O3 -Wall $(LTOFLAG)
release: LDFLAGS+= $(LTOFLAG)
release: $(RELEASEDIR)/$(NAME)
	
%/$(NAME): %/gparse %/drivers %/main.o
	$(CXX) $(@D)/main.o $(@D)/gparse/gparse.o $(@D)/drivers/drivers.o -o $@ $(CFLAGS)
	cp $@ $(BUILDROOT)/$(NAME)
	
%.dir:
	@mkdir -p $(@D)
$(DEBUGDIR)/%.o: %.cpp $(DEBUGDIR)/%.dir
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
$(RELEASEDIR)/%.o: %.cpp $(RELEASEDIR)/%.dir
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
	
%/drivers: %/drivers/driver.o %/drivers/kossel.o
	$(LD) -r $^ -o $(@D)/drivers/drivers.o $(LDFLAGS)

%/gparse: %/gparse/command.o %/gparse/event.o %/gparse/scheduler.o %/gparse/serial.o %/gparse/state.o
	$(LD) -r $^ -o $(@D)/gparse/gparse.o $(LDFLAGS)
	
#Prevent the automatic deletion of "intermediate" .o files by nulling .SECONDARY as follows.
.SECONDARY:

.PHONY: clean debug release
clean: 
	rm -rf $(BUILDROOT)
