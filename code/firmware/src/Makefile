#directory containing this makefile:
SRCDIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

CXX=g++
#Check if using gcc >= 4.7; Then we can use -flto flag. 4.6 and lower support lto, but have errors (specifically when "functional" is included)
GCC_GTEQ_470 := $(shell expr `$(CXX) -dumpversion | sed -e 's/\.\([0-9][0-9]\)/\1/g' -e 's/\.\([0-9]\)/0\1/g' -e 's/^[0-9]\{3,4\}$$/&00/'` \>= 40700)
#Check if we are compiling ON a raspberry pi. Ideally, we would check if the compilation target is a raspberry pi, but this is difficult.
#If the target arch is raspberry pi, then we can use direct access to the system clock, rather than kernel calls (faster)
#TARGET_RPI := $(shell expr `grep -o BCM2708 /proc/cpuinfo` == BCM2708)
#ifeq "$(shell expr `grep -o buildd /proc/version`)" "buildd"
ifeq "$(shell grep -o BCM2708 /proc/cpuinfo)" "BCM2708"
	TARGET_RPI := 1
else
	TARGET_RPI := 0
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
MACHINE=KosselPi
MACHINE_PATH=drivers/machines/$(shell echo $(MACHINE) | tr A-Z a-z).h
#fno-rounding-math lets compiler round floats either towards 0 or round down - whatever is most efficient
#fno-signed-zeros lets gcc treat -0.0 the same as +0.0
#freciprocal-math turns division-by-constant into multiplication-by-constant
#fsingle-precision-constant allows math involving literals to avoid promotion to double (eg x += 0.1 becomes c += 0.1f) (But this causes compile errors in glibc)
#fassociative-math allows x*y + z*y to become (x+z)*y. Also x+1 == y becomes x == y-1
#fno-math-errno allows gcc to forget about setting the global ERRNO for math routines like sqrt when passed a negative number. This allows inlining of the sqrt function, and I don't THINK errno is read by any glibc code.
UNSAFEOPT=-fno-signaling-nans -fno-rounding-math -fno-signed-zeros -freciprocal-math -fno-math-errno
#gcc <= 4.6 has c++11 support, but doesn't recognize the c++11 flag, so use -std=c++0x.
#the bcm2835 library has some pedantic errors, so use f=-fpermissive
CFLAGS=-std=c++0x -pthread -fpermissive -I$(SRCDIR) -Wall -Wextra -Wwrite-strings -Wno-unused-result -Wno-pmf-conversions $(LOGFLAGS) $(UNSAFEOPT) $(PROFFLAGS) $(LTOFLAG) $(ARCHFLAGS) -DMACHINE=$(MACHINE) -DMACHINE_PATH='"$(MACHINE_PATH)"' -DDTARGET_RPI=$(TARGET_RPI)
#CFLAGS+= -fassociateive-math #Allows a*a*a*a to become (a*a)*(a*a) for floating point.
LD=ld
LDFLAGS= $(LTOFLAG)
LIBS=-lrt
BUILDROOT=../build
DEBUGDIR=$(BUILDROOT)/debug
RELEASEDIR=$(BUILDROOT)/release
PROFILEDIR=$(BUILDROOT)/prof
MINSIZEDIR=$(BUILDROOT)/minsize
NAME=kossel-firmware

all: debug

-include $(wildcard *.d */*.d */*/*.d) #include auto-generated dependencies

#Configure debug/release settings:
# -fsanitize=address (gcc 4.8)
debug: CFLAGS+=-O0 -ggdb3 -fno-omit-frame-pointer
debug: $(DEBUGDIR)/$(NAME)
release: CFLAGS+= -O3 -ggdb3 -fno-omit-frame-pointer -fomit-frame-pointer
release: $(RELEASEDIR)/$(NAME)
profile: CFLAGS+= -O3 -DDRUNNING_IN_VM -pg
profile: $(PROFILEDIR)/$(NAME)
#defining NDEBUG removes assertions.
minsize: CFLAGS+= -DNDEBUG -Os -s -fomit-frame-pointer -ffunction-sections -fdata-sections -Wl,--gc-sections
minsize: LOGFLAGS+= -DDNO_LOGGING
minsize: $(MINSIZEDIR)/$(NAME)
	@strip --strip-all --remove-section=.comment --remove-section=.note $(BUILDROOT)/$(NAME)
	
%/$(NAME): %/common/common.a %/drivers/drivers.a %/gparse/gparse.a %/rest.a
	@echo $(GCC_GTEQ_470)
	$(CXX) $^ -o $@ $(CFLAGS) $(LIBS)
	cp $@ $(BUILDROOT)/$(NAME)
	
%.dir:
	@mkdir -p $(@D)
$(DEBUGDIR)/%.o: %.cpp $(DEBUGDIR)/%.dir
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
	
%/drivers/rpi/rpi.a: %/drivers/rpi/bcm2835.o %/drivers/rpi/rpi.o
	$(LD) -r $^ -o $@ $(LDFLAGS)
	
%/drivers/drivers.a: %/drivers/axisstepper.o %/drivers/driver.o %/drivers/rpi/rpi.a
	$(LD) -r $^ -o $@ $(LDFLAGS)

%/gparse/gparse.a: %/gparse/com.o %/gparse/command.o
	$(LD) -r $^ -o $@ $(LDFLAGS)
	
%/common/common.a: %/common/intervaltimer.o %/common/logging.o
	$(LD) -r $^ -o $@ $(LDFLAGS)
	
%/rest.a: %/argparse.o %/event.o %/main.o %/scheduler.o 
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
