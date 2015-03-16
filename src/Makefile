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

#Allow user to pass flags related to logging defaults:
LOGFLAGS=-DDNO_LOG_M105
DEFINES:=$(DEFINES) $(LOGFLAGS)

#Allow user to pass USE_PTHREAD=0 for a system that doesn't support pthreads (it's only used for upping the priority)
ifneq "$(USE_PTHREAD)" "0"
	DEFINES:=$(DEFINES) -DDUSE_PTHREAD
else
	NAME_EXT:=-NO_PTHREAD$(NAME_EXT)
endif

ifeq "$(ENABLE_TESTS)" "1"
	DEFINES:=$(DEFINES) -DDENABLE_TESTS
	NAME_EXT:=-ENABLE_TESTS$(NAME_EXT)
	LIBS:=$(LIBS) -pthread
endif

#gcc < 4.9 doesn't support colorized diagnostics (error messages)
ifeq "$(GCC_GTEQ_490)" "1"
	DIAGFLAG=-fdiagnostics-color=auto
endif

ifeq "$(GCC_GTEQ_470)" "1"
	DEFAULT_LTOFLAG=-flto #Enable link-time optimization
	ARCHFLAGS=-march=native -mtune=native
else #gcc < 4.7, or non-gcc compiler
	DEFAULT_LTOFLAG= #Disable link-time optimization, as it often leads to link-time errors for gcc < 4.8
	ARCHFLAGS= #gcc-4.6 on arm doesn't support march=native
endif

#if no LTO flag present, then use the default. Note: the default may vary based on debug/release build
LTOFLAG=$(DEFAULT_LTOFLAG)

#Default machine if not given via command line argument:
MACHINE=rpi/firepickdelta.h
ifeq "$(suffix $(MACHINE))" ".h"
	MACHINEPARSED:=$(basename $(MACHINE))
else
	MACHINEPARSED:=$(MACHINE)
endif
#Allow for one to specify either the full path (machines/<platform>/<machine>.h) OR a partial path, without the machines prefix (<platform>/machine.h)
MACHINEPARSED:=$(MACHINEPARSED:machines/%=%)
MACHINECPP=$(subst /,::,$(MACHINEPARSED))


MACHINE_ENDPATH=$(subst ::,/,$(MACHINEPARSED))
MACHINE_PATH=machines/$(MACHINE_ENDPATH).h
PLATFORM=$(subst /,,$(dir $(MACHINE_ENDPATH)))
PLATFORM_DIR=platforms/$(PLATFORM)

#Check for platform-specific file overrides:
ifneq ("$(wildcard platforms/$(PLATFORM)/chronoclock.h)","")
    PLAT_DEFINES:=$(PLAT_DEFINES) -DPLATFORM_DRIVER_CHRONOCLOCK='"platforms/$(PLATFORM)/chronoclock.h"'
endif
ifneq ("$(wildcard platforms/$(PLATFORM)/hardwarescheduler.h)","")
    PLAT_DEFINES:=$(PLAT_DEFINES) -DPLATFORM_DRIVER_HARDWARESCHEDULER='"platforms/$(PLATFORM)/hardwarescheduler.h"'
endif
ifneq ("$(wildcard platforms/$(PLATFORM)/thisthreadsleep.h)","")
    PLAT_DEFINES:=$(PLAT_DEFINES) -DPLATFORM_DRIVER_THISTHREADSLEEP='"platforms/$(PLATFORM)/thisthreadsleep.h"'
endif
ifneq ("$(wildcard platforms/$(PLATFORM)/primitiveiopin.h)","")
    PLAT_DEFINES:=$(PLAT_DEFINES) -DPLATFORM_DRIVER_PRIMITIVEIOPIN='"platforms/$(PLATFORM)/primitiveiopin.h"'
endif
PLAT_DEFINES:=$(PLAT_DEFINES) -DMACHINE='$(MACHINECPP)' -DMACHINE_PATH='"$(MACHINE_PATH)"' -DDTARGET_PLATFORM_LOWER="$(PLATFORM)"
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
INCFLAGS=-I$(SRCDIR) -I$(SRCDIR)/../lib
STDFLAGS=-std=c++0x
WARNFLAGS= -Wall -Wextra -Wundef -Wwrite-strings -Wno-unused-result
CFLAGS=$(STDFLAGS) $(WARNFLAGS) $(INCFLAGS) $(DIAGFLAG) $(LTOFLAG) $(UNSAFEOPT) $(ARCHFLAGS) $(DEFINES) $(PLAT_DEFINES) -DBUILD_TYPE_$(TARGET)
#Need the --no-as-needed flag to fix bug in some gcc versions when compiling with multithreading (for tests): http://stackoverflow.com/questions/19463602/compiling-multithread-code-with-g
LDFLAGS=-Wl,--no-as-needed
#Extra flags to apply JUST to main.cpp (eg -fwhole-program)
MAINCFLAGS=
#-lrt is the realtime posix library. Appears to be needed for things like clock_nanosleep
LIBS:=$(LIBS) -lrt
BUILDROOT=../build
#NAMELINK will become a symbolic link to the actual binary
NAMELINK=printipi
#NAME_EXT is the version-info that should be tagged onto the binary name
#Note: explicitly prefix with PLATFORM, because it's possible to build (eg) a rpi Machine with the generic platform implementation
NAME_EXT:=-$(PLATFORM)-$(subst /,-,$(MACHINEPARSED))-$(CXX)$(NAME_EXT)
#NAME of binary file:
NAME=$(NAMELINK)$(NAME_EXT)

DEBUGDIR_BASE=$(BUILDROOT)/debug
DEBUGRELDIR_BASE=$(BUILDROOT)/debugrel
RELEASEDIR_BASE=$(BUILDROOT)/release
PROFILEDIR_BASE=$(BUILDROOT)/prof
MINSIZEDIR_BASE=$(BUILDROOT)/minsize

DEBUGDIR=$(DEBUGDIR_BASE)$(NAME_EXT)
DEBUGRELDIR=$(DEBUGRELDIR_BASE)$(NAME_EXT)
RELEASEDIR=$(RELEASEDIR_BASE)$(NAME_EXT)
PROFILEDIR=$(PROFILEDIR_BASE)$(NAME_EXT)
MINSIZEDIR=$(MINSIZEDIR_BASE)$(NAME_EXT)

SOURCES=$(wildcard *.cpp common/**.cpp gparse/**.cpp iodrivers/**.cpp machines/*.cpp motion/**.cpp $(PLATFORM_DIR)/**.cpp)
OBJECTS=$(SOURCES:%.cpp=%.o)
DEPFILES=$(SOURCES:%.cpp=%.d)

all: debugrel

#include auto-generated dependencies:
-include $(DEPFILES)

#Configure debug/release settings:
debug: TARGET=debug
debug: DEFAULT_LTOFLAG=
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

	
%/$(NAME): $(addprefix %/,$(OBJECTS))
	$(CXX) $^ -o $@ $(CFLAGS) $(LDFLAGS) $(MAINCFLAGS) $(LIBS)
	cp $@ $(BUILDROOT)/$(NAME)-$(TARGET)
	ln -f -s $(BUILDROOT)/$(NAME)-$(TARGET) $(BUILDROOT)/$(NAMELINK)
	
$(DEBUGDIR)/%.o $(DEBUGRELDIR)/%.o $(RELEASEDIR)/%.o $(PROFILEDIR)/%.o $(MINSIZEDIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) -MM -MP -MT $@ -MT $*.d $(CFLAGS) $< > $*.d
	$(CXX) -c -o $@ $*.cpp $(CFLAGS)

#main.cpp dynamically #includes the MACHINE, so we want to make that an explicit dependency.	
%/main.cpp: $(MACHINE_PATH)

#Make documentation:
doc: TARGET=doc
doc:
	rm -rf ../doc
	@mkdir -p ../doc
	cldoc generate $(STDFLAGS) $(INCFLAGS) $(DEFINES) -DMACHINE_PATH='"machines/generic/cartesian.h"' -DMACHINE="generic::cartesian" -- --report --merge docs --output ../doc `find . -type f -name '*.cpp'` `find . -type f -name '*.h'`
	
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
	rm -rf `find . -type f -name '*.d'` `find $(PLATFORM_DIR) -type f -name '*.d'`
