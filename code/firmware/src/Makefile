#NOTE: compiling with g++-4.7.2 may lead to some runtime errors in the scheduler!

#directory containing this makefile:
SRCDIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

CXX=g++
GCC_GTEQ_470 := $(shell expr `$(CXX) -dumpversion | sed -e 's/\.\([0-9][0-9]\)/\1/g' -e 's/\.\([0-9]\)/0\1/g' -e 's/^[0-9]\{3,4\}$$/&00/'` \>= 40700)
LOGFLAGS=-DDNO_LOG_M105
#LOGFLAGS= -DNO_LOGGING -DNO_USAGE_INFO
CFLAGS=-std=c++0x -pthread -fpermissive -I$(SRCDIR) -Wall -Wextra -Wwrite-strings -Wno-unused-result -Wno-pmf-conversions $(LOGFLAGS)
#CFLAGS=-std=c++0x -Og -g -flto -fsanitize=address -fno-omit-frame-pointer #Use c++0x for gcc <= 4.6, not c++11
#CFLAGS+= -fassociateive-math #Allows a*a*a*a to become (a*a)*(a*a) for floating point.
LD=ld
LDFLAGS=
LIBS=-lrt
ifeq "$(GCC_GTEQ_470)" "1"
	LTOFLAG=-flto #Enable link-time optimization
	ARCHFLAGS=-march=native -mtune=native
else #gcc < 4.7, or non-gcc compiler
	LTOFLAG= #Disable link-time optimization, as it often leads to link-time errors for gcc < 4.8
	ARCHFLAGS= #gcc-4.6 on arm doesn't support march=native
endif
BUILDROOT=../build
DEBUGDIR=$(BUILDROOT)/debug
RELEASEDIR=$(BUILDROOT)/release
MINSIZEDIR=$(BUILDROOT)/minsize
NAME=kossel-firmware

all: debug

-include $(wildcard **.d)

#Configure debug/release settings:
# -fsanitize=address (gcc 4.8)
debug: CFLAGS+=-O0 -ggdb3 -fno-omit-frame-pointer $(ARCHFLAGS)
debug: $(DEBUGDIR)/$(NAME)
release: CFLAGS+= -O3 $(LTOFLAG) $(ARCHFLAGS)
release: LDFLAGS+= $(LTOFLAG)
release: $(RELEASEDIR)/$(NAME)
minsize: CFLAGS+= -Os -s -fomit-frame-pointer -ffunction-sections -fdata-sections -Wl,--gc-sections $(LTOFLAG) $(ARCHFLAGS)
minsize: LDFLAGS+= $(LTOFLAG)
minsize: LOGFLAGS+= -DDNO_LOGGING
minsize: $(MINSIZEDIR)/$(NAME)
	@strip --strip-all --remove-section=.comment --remove-section=.note $(BUILDROOT)/$(NAME)
	
%/$(NAME): %/gparse/gparse.a %/drivers/drivers.a %/rest.a
	@echo $(GCC_GTEQ_470)
	$(CXX) $^ -o $@ $(CFLAGS) $(LIBS)
	cp $@ $(BUILDROOT)/$(NAME)
	
%.dir:
	@mkdir -p $(@D)
$(DEBUGDIR)/%.o: %.cpp %.d $(DEBUGDIR)/%.dir
	$(CXX) -MM -MP -MT $@ -MT $*.d $(CFLAGS) $< > $*.d
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
$(RELEASEDIR)/%.o: %.cpp $(RELEASEDIR)/%.dir
	$(CXX) -MM -MP -MT $@ -MT $*.d $(CFLAGS) $< > $*.d
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
$(MINSIZEDIR)/%.o: %.cpp $(MINSIZEDIR)/%.dir
	$(CXX) -MM -MP -MT $@ -MT $*.d $(CFLAGS) $< > $*.d
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
	
%/drivers/rpi/rpi.a: %/drivers/rpi/bcm2835.o %/drivers/rpi/rpi.o
	$(LD) -r $^ -o $@ $(LDFLAGS)
	
%/drivers/drivers.a: %/drivers/axisstepper.o %/drivers/driver.o %/drivers/rpi/rpi.a
	$(LD) -r $^ -o $@ $(LDFLAGS)

%/gparse/gparse.a: %/gparse/com.o %/gparse/command.o
	$(LD) -r $^ -o $@ $(LDFLAGS)
	
%/rest.a: %/argparse.o %/event.o %/intervaltimer.o %/logging.o %/main.o %/scheduler.o %/timeutil.o
	$(LD) -r $^ -o $@ $(LDFLAGS)
	
#Prevent the automatic deletion of "intermediate" .o files by nulling .SECONDARY as follows.
.SECONDARY:

.PHONY: clean cleandebug cleanrelease cleanminsize debug release minsize
cleandebug:
	rm -rf $(DEBUGDIR)
cleanrelease:
	rm -rf $(RELEASEDIR)
cleanminsize:
	rm -rf $(MINSIZEDIR)
clean: cleandebug cleanrelease cleanminsize
	rm -rf $(BUILDROOT)/$(NAME)
