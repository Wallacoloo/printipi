CXX=g++
CFLAGS=-std=c++0x -Wall -Wextra -Wwrite-strings -Wno-unused-result
#CFLAGS=-std=c++0x -Og -g -flto -fsanitize=address -fno-omit-frame-pointer #Use c++0x for gcc <= 4.6, not c++11
LD=ld
LDFLAGS=
#FLTOFLAG=-flto #Enable link-time optimization
LTOFLAG= #Disable link-time optimization, as it often leads to link-time errors for gcc < 4.8
BUILDROOT=../build
DEBUGDIR=$(BUILDROOT)/debug
RELEASEDIR=$(BUILDROOT)/release
MINSIZEDIR=$(BUILDROOT)/minsize
NAME=kossel-firmware
#ARCHFLAGS=-march=native -mtune=native
ARCHFLAGS= #gcc-4.6 on arm doesn't support march=native

all: debug

#Configure debug/release settings:
# -fsanitize=address (gcc 4.8)
debug: CFLAGS+=-O2 -g -fno-omit-frame-pointer
debug: $(DEBUGDIR)/$(NAME)
release: CFLAGS+= -O3 $(LTOFLAG) $(ARCHFLAGS)
release: LDFLAGS+= $(LTOFLAG)
release: $(RELEASEDIR)/$(NAME)
minsize: CFLAGS+= -Os -fomit-frame-pointer $(LTOFLAG) $(ARCHFLAGS)
minsize: LDFLAGS+= $(LTOFLAG)
minsize: $(MINSIZEDIR)/$(NAME)
	@strip -s $(BUILDROOT)/$(NAME)
	
%/$(NAME): %/gparse/gparse.a %/drivers/drivers.a %/rest.a
	$(CXX) $^ -o $@ $(CFLAGS)
	cp $@ $(BUILDROOT)/$(NAME)
	
%.dir:
	@mkdir -p $(@D)
$(DEBUGDIR)/%.o: %.cpp $(DEBUGDIR)/%.dir
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
$(RELEASEDIR)/%.o: %.cpp $(RELEASEDIR)/%.dir
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
$(MINSIZEDIR)/%.o: %.cpp $(MINSIZEDIR)/%.dir
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
	
%/drivers/drivers.a: %/drivers/axisstepper.o %/drivers/driver.o %/drivers/kossel.o
	$(LD) -r $^ -o $@ $(LDFLAGS)

%/gparse/gparse.a: %/gparse/command.o %/gparse/serial.o
	$(LD) -r $^ -o $@ $(LDFLAGS)
	
%/rest.a: %/event.o %/main.o %/scheduler.o %/state.o
	$(LD) -r $^ -o $@ $(LDFLAGS)
	
#Prevent the automatic deletion of "intermediate" .o files by nulling .SECONDARY as follows.
.SECONDARY:

.PHONY: clean debug release
clean: 
	rm -rf $(BUILDROOT)
