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
GCC_GTEQ_490 := $(shell expr `$(CXX) -dumpversion | sed -e 's/\.\([0-9][0-9]\)/\1/g' -e 's/\.\([0-9]\)/0\1/g' -e 's/^[0-9]\{3,4\}$$/&00/'` \>= 40900)
#Allow user to pass USE_PTHREAD=0 for a system that doesn't support pthreads (it's only used for upping the priority)
ifneq "$(USE_PTHREAD)" "0"
    USE_PTHREAD := 1
endif
LOGFLAGS=-DDNO_LOG_M105
PROFFLAGS=
ifeq "$(GCC_GTEQ_490)" "1"
	DIAGFLAG=-fdiagnostics-color=auto
else
	DIAGFLAG= #gcc < 4.9 doesn't support colorized diagnostics (error messages)
endif
ifeq "$(GCC_GTEQ_470)" "1"
	LTOFLAG=-flto #Enable link-time optimization
	ARCHFLAGS=-march=native -mtune=native
else #gcc < 4.7, or non-gcc compiler
	LTOFLAG= #Disable link-time optimization, as it often leads to link-time errors for gcc < 4.8
	ARCHFLAGS= #gcc-4.6 on arm doesn't support march=native
endif

#Default machine if not given via command line argument:
MACHINE=rpi/kosselrampsfd.h
ifeq "$(suffix $(MACHINE))" ".h"
	MACHINEPARSED:=$(basename $(MACHINE))
else
	MACHINEPARSED:=$(MACHINE)
endif
MACHINECPP=$(subst /,::,$(MACHINEPARSED))


MACHINE_ENDPATH=$(subst ::,/,$(MACHINEPARSED))
MACHINE_PATH=machines/$(MACHINE_ENDPATH).h
MACHINE_CLASS=$(subst /,,$(dir $(MACHINE_ENDPATH)))
MACHINE_CLASS_DIR=platforms/$(MACHINE_CLASS)

#Check for platform-specific file overrides:
ifneq ("$(wildcard platforms/$(MACHINE_CLASS)/chronoclock.h)","")
    DEFINES:=$(DEFINES) -DPLATFORM_DRIVER_CHRONOCLOCK='"platforms/$(MACHINE_CLASS)/chronoclock.h"'
endif
ifneq ("$(wildcard platforms/$(MACHINE_CLASS)/hardwarescheduler.h)","")
    DEFINES:=$(DEFINES) -DPLATFORM_DRIVER_HARDWARESCHEDULER='"platforms/$(MACHINE_CLASS)/hardwarescheduler.h"'
endif
ifneq ("$(wildcard platforms/$(MACHINE_CLASS)/thisthreadsleep.h)","")
    DEFINES:=$(DEFINES) -DPLATFORM_DRIVER_THISTHREADSLEEP='"platforms/$(MACHINE_CLASS)/thisthreadsleep.h"'
endif
ifneq ("$(wildcard platforms/$(MACHINE_CLASS)/primitiveiopin.h)","")
    DEFINES:=$(DEFINES) -DPLATFORM_DRIVER_PRIMITIVEIOPIN='"platforms/$(MACHINE_CLASS)/primitiveiopin.h"'
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
CFLAGS=-std=c++0x -I$(SRCDIR) -Wall -Wextra -Wwrite-strings -Wno-unused-result $(LOGFLAGS) $(PROFFLAGS) $(DIAGFLAG) $(LTOFLAG) $(UNSAFEOPT) $(ARCHFLAGS) -DMACHINE='$(MACHINECPP)' -DMACHINE_PATH='"$(MACHINE_PATH)"' -DDTARGET_PLATFORM_LOWER="$(MACHINE_CLASS)" -DDUSE_PTHREAD=$(USE_PTHREAD) $(DEFINES) -DBUILD_TYPE_$(TARGET)
#Extra flags to apply JUST to main.cpp (eg -fwhole-program)
MAINCFLAGS=
#Use the C++ compiler to generate archives, instead of AR or LD, so as to be compatible with clang
MAKEARCHIVE=$(CXX) -Wl,-r -nostdlib $(LTOFLAG) -o
LDFLAGS= $(LTOFLAG)
#-lrt is the realtime posix library. Appears to be needed for things like clock_nanosleep
LIBS=-lrt
BUILDROOT=../build
#NAMELINK will become a symbolic link to the actual binary
NAMELINK=printipi
#NAME_EXT is the version-info that should be tagged onto the binary name
#Note: explicitly prefix with MACHINE_CLASS, because it's possible to build (eg) a rpi Machine with the generic platform implementation
NAME_EXT=$(MACHINE_CLASS)-$(subst /,-,$(MACHINEPARSED))-$(CXX)
#NAME of binary file:
NAME=$(NAMELINK)-$(NAME_EXT)

DEBUGDIR_BASE=$(BUILDROOT)/debug
DEBUGRELDIR_BASE=$(BUILDROOT)/debugrel
RELEASEDIR_BASE=$(BUILDROOT)/release
PROFILEDIR_BASE=$(BUILDROOT)/prof
MINSIZEDIR_BASE=$(BUILDROOT)/minsize

DEBUGDIR=$(DEBUGDIR_BASE)-$(NAME_EXT)
DEBUGRELDIR=$(DEBUGRELDIR_BASE)-$(NAME_EXT)
RELEASEDIR=$(RELEASEDIR_BASE)-$(NAME_EXT)
PROFILEDIR=$(PROFILEDIR_BASE)-$(NAME_EXT)
MINSIZEDIR=$(MINSIZEDIR_BASE)-$(NAME_EXT)



all: debugrel

#include auto-generated dependencies:
-include $(wildcard *.d */*.d */*/*.d */*/*/*.d $(MACHINE_CLASS_DIR)/*.d $(MACHINE_CLASS_DIR)/*/*.d)

#Configure debug/release settings:
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

#Sometimes need an empty dependency in order to avoid 'no input file' errors, etc:
%/empty.cpp:
	touch $@
	
%/$(NAME): %/common/common.a %/iodrivers/iodrivers.a %/gparse/gparse.a %/motion/motion.a %/rest.a %/$(MACHINE_CLASS_DIR)/$(MACHINE_CLASS).a
	$(CXX) $^ -o $@ $(CFLAGS) $(MAINCFLAGS) $(LIBS)
	cp $@ $(BUILDROOT)/$(NAME)-$(TARGET)
	ln -f -s $(BUILDROOT)/$(NAME)-$(TARGET) $(BUILDROOT)/$(NAMELINK)
	
$(DEBUGDIR)/%.o $(DEBUGRELDIR)/%.o $(RELEASEDIR)/%.o $(PROFILEDIR)/%.o $(MINSIZEDIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) -MM -MP -MT $@ -MT $*.d $(CFLAGS) $< > $*.d
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)
	
#Detect and build platform dependencies
#  Thus, if you're targeting the Raspberry Pi (rpi), any source files under platforms/rpi/* will be compiled into the executable
%/$(MACHINE_CLASS_DIR)/$(MACHINE_CLASS).a: $(addprefix %/,$(patsubst %.cpp,%.o,$(wildcard $(MACHINE_CLASS_DIR)/*.cpp))) %/empty.o
	@mkdir -p $(@D)
	$(MAKEARCHIVE) $@ $^
	
%/iodrivers/iodrivers.a: %/iodrivers/iopin.o
	$(MAKEARCHIVE) $@ $^

%/gparse/gparse.a: %/gparse/com.o %/gparse/command.o %/gparse/response.o
	$(MAKEARCHIVE) $@ $^
	
%/common/common.a: %/common/logging.o
	$(MAKEARCHIVE) $@ $^

%/motion/motion.a: %/motion/axisstepper.o
	$(MAKEARCHIVE) $@ $^
	
%/rest.a: %/argparse.o %/filesystem.o %/main.o %/pid.o %/schedulerbase.o 
	$(MAKEARCHIVE) $@ $^
%/main.cpp: $(MACHINE_PATH)

#Make documentation:
doc:
	rm -rf ../doc
	@mkdir -p ../doc
	cldoc generate -std=c++0x -I. -DMACHINE_PATH='"machines/generic/cartesian.h"' -DMACHINE="generic::cartesian" -- --report --merge docs --output ../doc **/*.cpp **/*.h
	
#Prevent the automatic deletion of "intermediate" .o files after the build by nulling .SECONDARY as follows.
.SECONDARY:

.PHONY: clean cleandebug cleanrelease cleanprofile cleanminsize debug debugrel release profile minsize doc
cleandebug:
	rm -rf $(DEBUGDIR_BASE)-*
cleandebugrel:
	rm -rf $(DEBUGRELDIR_BASE)-*
cleanrelease:
	rm -rf $(RELEASEDIR_BASE)-*
cleanprofile:
	rm -rf $(PROFILEDIR_BASE)-*
cleanminsize:
	rm -rf $(MINSIZEDIR_BASE)-*
clean: cleandebug cleandebugrel cleanrelease cleanprofile cleanminsize
	rm -rf $(wildcard *.d */*.d */*/*.d */*/*/*.d $(MACHINE_CLASS_DIR)/*.d $(MACHINE_CLASS_DIR)/*/*.d)
