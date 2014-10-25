##USAGE:
## make [MACHINE=<machine>] [<buildtype>] [DEFINES=<defines>]
##   <machine> is the case-sensitive c++ class name of the machine you wish to target. eg rpi::KosselPi or generic::Example
##   <buildtype> = `release' or `debug' or `debugrel' or `profile` or `minsize'. Defaults to debug
##   <defines> is a series of (define-related) flags to send to the C compiler. Eg DEFINES=-DNDEBUG


#directory containing this makefile:
SRCDIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

CXX=g++
#Check if using gcc >= 4.7; Then we can use -flto flag. 4.6 and lower support lto, but have errors (specifically when "functional" is included)
GCC_GTEQ_470 := $(shell expr `$(CXX) -dumpversion | sed -e 's/\.\([0-9][0-9]\)/\1/g' -e 's/\.\([0-9]\)/0\1/g' -e 's/^[0-9]\{3,4\}$$/&00/'` \>= 40700)
#Allow user to pass USE_PTHREAD=0 for a system that doesn't support pthreads (it's only used for upping the priority)
ifneq "$(USE_PTHREAD)" "0"
    USE_PTHREAD := 1
endif
LOGFLAGS=-DDNO_LOG_M105
PROFFLAGS=
#LOGFLAGS= -DNO_LOGGING -DNO_USAGE_INFO
ifeq "$(GCC_GTEQ_470)" "1"
	LTOFLAG=-flto #Enable link-time optimization
	ARCHFLAGS=-march=native -mtune=native
else #gcc < 4.7, or non-gcc compiler
	LTOFLAG= #Disable link-time optimization, as it often leads to link-time errors for gcc < 4.8
	ARCHFLAGS= #gcc-4.6 on arm doesn't support march=native
endif
#Default machine if not given via command line argument:
MACHINE=rpi::KosselPi
MACHINECPP=$(subst /,::,$(MACHINE))
LOWERMACHINE=$(subst ::,/,$(shell echo $(MACHINE) | tr A-Z a-z))
MACHINE_PATH=machines/$(LOWERMACHINE).h
LOWER_MACHINE_CLASS=$(subst /,,$(dir $(LOWERMACHINE)))
MACHINE_CLASS=$(shell echo $(LOWER_MACHINE_CLASS) | tr a-z A-Z)
#Check for platform-specific file overrides:
ifneq ("$(wildcard drivers/$(LOWER_MACHINE_CLASS)/chronoclock.h)","")
    DEFINES:=$(DEFINES) -DPLATFORM_DRIVER_CHRONOCLOCK='"drivers/$(LOWER_MACHINE_CLASS)/chronoclock.h"'
endif
ifneq ("$(wildcard drivers/$(LOWER_MACHINE_CLASS)/hardwarescheduler.h)","")
    DEFINES:=$(DEFINES) -DPLATFORM_DRIVER_HARDWARESCHEDULER='"drivers/$(LOWER_MACHINE_CLASS)/hardwarescheduler.h"'
endif
#fno-rounding-math lets compiler round floats either towards 0 or round down - whatever is most efficient
#fno-signed-zeros lets gcc treat -0.0 the same as +0.0
#freciprocal-math turns division-by-constant into multiplication-by-constant
#fsingle-precision-constant allows math involving literals to avoid promotion to double (eg x += 0.1 becomes c += 0.1f) (But this causes compile errors in glibc)
#fassociative-math allows x*y + z*y to become (x+z)*y. Also x+1 == y becomes x == y-1
#fno-math-errno allows gcc to forget about setting the global ERRNO for math routines like sqrt when passed a negative number. This allows inlining of the sqrt function, and I don't THINK errno is read by any glibc code.
#fmerge-all-constants allows eg a const float and a const array item with same binary value to share the same address. *may* cause issues with recursive functions?
#-fweb allows local variables to not have a 'home' register - more efficient partitioning, but also more difficult to debug.
UNSAFEOPT=-fno-signaling-nans -fno-rounding-math -fno-signed-zeros -freciprocal-math -fno-math-errno 
#gcc <= 4.6 has c++11 support, but doesn't recognize the c++11 flag, so use -std=c++0x.
CFLAGS=-std=c++0x -I$(SRCDIR) -Wall -Wextra -Wwrite-strings -Wno-unused-result -Wno-pmf-conversions $(LOGFLAGS) $(PROFFLAGS) $(LTOFLAG) $(UNSAFEOPT) $(ARCHFLAGS) -DMACHINE='$(MACHINECPP)' -DMACHINE_PATH='"$(MACHINE_PATH)"' -DDTARGET_PLATFORM_$(MACHINE_CLASS) -DDTARGET_PLATFORM_LOWER="$(LOWER_MACHINE_CLASS)" -DDUSE_PTHREAD=$(USE_PTHREAD) $(DEFINES)
MAINCFLAGS=#-fwhole-program
#CFLAGS+= -fassociative-math #Allows a*a*a*a to become (a*a)*(a*a) for floating point.
LD=ld
LDFLAGS= $(LTOFLAG)
LIBS=-lrt
BUILDROOT=../build
DEBUGDIR=$(BUILDROOT)/debug
DEBUGRELDIR=$(BUILDROOT)/debugrel
RELEASEDIR=$(BUILDROOT)/release
PROFILEDIR=$(BUILDROOT)/prof
MINSIZEDIR=$(BUILDROOT)/minsize
#name of binary file:
NAME=printipi-$(subst /,-,$(LOWERMACHINE))
#NAMELINK will become a symbolic link to the actual binary
NAMELINK=printipi

all: debug

#include auto-generated dependencies:
-include $(wildcard *.d */*.d */*/*.d */*/*/*.d)

#Configure debug/release settings:
# -fsanitize=address (gcc 4.8)
debug: TARGET=debug
debug: CFLAGS+=-O0 -ggdb3 -fno-omit-frame-pointer
debug: $(DEBUGDIR)/$(NAME)
#optimized debug build, suitable for using with gdb, perf, etc:
debugrel: TARGET=debugrel
debugrel: CFLAGS+= -O3 -ggdb3 -fno-omit-frame-pointer
debugrel: $(DEBUGRELDIR)/$(NAME)
#release build:
release: TARGET=release
release: CFLAGS+= -O3 -ggdb3 -fomit-frame-pointer
release: $(RELEASEDIR)/$(NAME)
#optimized debug mode with less time sensitivity. Can run under valgrind, etc:
profile: TARGET=profile
profile: CFLAGS+= -O3 -DDRUNNING_IN_VM -ggdb3 -fno-omit-frame-pointer 
profile: $(PROFILEDIR)/$(NAME)
minsize: TARGET=minsize
#defining NDEBUG removes assertions.
minsize: CFLAGS+= -DNDEBUG -Os -s -fmerge-all-constants -fomit-frame-pointer -ffunction-sections -fdata-sections -Wl,--gc-sections
minsize: LOGFLAGS+= -DDNO_LOGGING
minsize: $(MINSIZEDIR)/$(NAME)
	@strip --strip-all --remove-section=.comment --remove-section=.note $(BUILDROOT)/$(NAME)-$(TARGET)
	
%/$(NAME): %/common/common.a %/drivers/drivers.a %/gparse/gparse.a %/rest.a
	$(CXX) $^ -o $@ $(CFLAGS) $(MAINCFLAGS) $(LIBS)
	cp $@ $(BUILDROOT)/$(NAME)-$(TARGET)
	ln -f -s $(BUILDROOT)/$(NAME)-$(TARGET) $(BUILDROOT)/$(NAMELINK)
	
%.dir:
	@mkdir -p $(@D)
$(DEBUGDIR)/%.o: %.cpp $(DEBUGDIR)/%.dir
	$(CXX) -MM -MP -MT $@ -MT $*.d $(CFLAGS) $< > $*.d
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
$(DEBUGRELDIR)/%.o: %.cpp $(DEBUGRELDIR)/%.dir
	$(CXX) -MM -MP -MT $@ -MT $*.d $(CFLAGS) $< > $*.d
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
$(RELEASEDIR)/%.o: %.cpp $(RELEASEDIR)/%.dir
	$(CXX) -MM -MP -MT $@ -MT $*.d $(CFLAGS) $< > $*.d
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
$(PROFILEDIR)/%.o: %.cpp $(PROFILEDIR)/%.dir
	$(CXX) -MM -MP -MT $@ -MT $*.d $(CFLAGS) $< > $*.d
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
$(MINSIZEDIR)/%.o: %.cpp $(MINSIZEDIR)/%.dir
	$(CXX) -MM -MP -MT $@ -MT $*.d $(CFLAGS) $< > $*.d
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
	
%/drivers/rpi/rpi.a: %/drivers/rpi/chronoclock.o %/drivers/rpi/hardwarescheduler.o %/drivers/rpi/mitpi.o
	$(LD) -r $^ -o $@ $(LDFLAGS)
	
%/drivers/drivers.a: %/drivers/axisstepper.o %/drivers/rpi/rpi.a
	$(LD) -r $^ -o $@ $(LDFLAGS)

%/gparse/gparse.a: %/gparse/com.o %/gparse/command.o %/gparse/response.o
	$(LD) -r $^ -o $@ $(LDFLAGS)
	
%/common/common.a: %/common/logging.o
	$(LD) -r $^ -o $@ $(LDFLAGS)
	
%/rest.a: %/argparse.o %/event.o %/filesystem.o %/main.o %/schedulerbase.o 
	$(LD) -r $^ -o $@ $(LDFLAGS)
%/main.cpp: $(MACHINE_PATH)
	
#Prevent the automatic deletion of "intermediate" .o files by nulling .SECONDARY as follows.
.SECONDARY:

.PHONY: clean cleandebug cleanrelease cleanprofile cleanminsize debug release profile minsize
cleandebug:
	rm -rf $(DEBUGDIR)
cleanrelease:
	rm -rf $(RELEASEDIR)
cleanprofile:
	rm -rf $(PROFILEDIR)
cleanminsize:
	rm -rf $(MINSIZEDIR)
clean: cleandebug cleanrelease cleanprofile cleanminsize
	rm -rf $(BUILDROOT)/$(NAME)
	rm -rf $(wildcard *.d */*.d */*/*.d)
