/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
 /*
 * processor documentation is at: http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 * pg 38 for DMA
 * pg 61 for DMA DREQ PERMAP
 * pg 89 for gpio
 * pg 119 for PCM
 * pg 138 for PWM
 * pg 172 for timer info
 * Addendum is http://www.scribd.com/doc/127599939/BCM2835-Audio-clocks
 *
 * A few annotations for GPIO/DMA/PWM are available here: https://github.com/626Pilot/RaspberryPi-NeoPixel-WS2812/blob/master/ws2812-RPi.c
 *   https://github.com/metachris/raspberrypi-pwm/blob/master/rpio-pwm/rpio_pwm.c
 *   https://github.com/richardghirst/PiBits/blob/master/ServoBlaster/user/servod.c
 *
 * Cache info can be found here: http://www.freelists.org/post/raspi-internals/caches,18
 *   0x00000000 - L1 & L2 cache
 *   0x40000000 - L2 cache coherent (ie L1 writes are propagated to L2?)
 *   0x80000000 - L2 cache only
 *   0xc0000000 - direct uncached
 *
 * Useful DMA timings, etc: http://www.raspberrypi.org/forums/viewtopic.php?f=37&t=7696&start=50
 *
 * The general idea is to have a buffer of N blocks, where each block is the same size as the gpio registers, 
 *   and have the DMA module continually copying the data in this buffer into those registers.
 * In this way, we can have (say) 32 blocks, and then be able to buffer the next 32 IO frames.
 *
 * How is DMA transfer rate controlled?
 * We can use the DREQ (data request) feature.
 *   PWM supports a configurable data consumption clock (defaults to 100MHz)
 *   PWM (and SPI, PCM) can fire a DREQ signal any time its fifo falls below a certain point.
 *   But we are never filling the FIFO, so DREQ would be permanently high.
 *   Could feed PWM with dummy data, and use 2 DMA channels (one to PWM, one to GPIO, both gated), but the write-time to GPIOs may vary from the PWM, so gating may be improper
 * Or we can use the WAITS portion of the CB header. This allows up to 31 cycle delay -> ~25MHz?
 *   Will have to manually determine timing characteristics though.
 * Or use 2 dma channels:
 *   Have one sending the data into PWM, which is DREQ limited
 *   Have another copying from PWM Fifo to GPIOs at a non-limited rate (or limited with WAITS). This is peripheral -> peripheral, so I think it will have its own data bus.
 *     Unfortunately, the destination can only be one word. Luckily, we have 2 PWM channels - one for setting & one for clearing GPIOs. All gpios that are broken out into the header are in the first register (verified)
 *   Sadly, it appears that the PWM FIFO cannot be read from. One can read the current PWM output, but only if the FIFO is disabled, in which case the DREQ is too. [Pg 146: DAT1 register: "This register stores the 32 bit data to be sent by the PWM Controller when USEFi is 0."]
 *   Maybe PCM fifo can be read from? PCM is unclear in this portion of the documentation. Pg 129: FIFO_A register: "This is the FIFO port of the PCM. Data written here is transmitted, and received data is read from here... Bits 0-31: Reserved: Write as 0, read as don't care". I suspect you cannot read data queued for transmission
 * Another possibility using 2 DMA channels:
 *   Find 2 unused (trash) registers around the PWM FIFO input. Write to these continually, paced through the FIFO.
 *   Have another channel (or 2) copy from these trash registers into GPCLR and GPSET, at a much higher frequency, only paced by WAITS.
 *   Note: register layout is: CONTROL, STATUS, DMAC, [unused - mapped?], CH1 RANGE, CH1 DATA, FIFO IN, [unused - mapped?], CH2 RANGE, CH2 DATA
 *     PCM register map: CONTROL/STATUS, FIFO DATA, PCM MODE, PCM RECV CONFIG, PCM TRANSMIT CONFIG, DMA REQ LEVEL, PCM INTERRUPT ENABLES, PCM INTERRUPT STATUS, PCM GRAY CODE CONTROL
 *   May be possible to use CH1/2 DATA, which are documented as not being used in FIFO mode.
 *   Or, investiage the unused registers (they likely read garbage).
 *   Lastly, it MAY be possible to even use the CH2 RANGE register if CH1 is disabled.
 *   Best option could very well be using CH2 RANGE and CH2 DATA registers
 *   Ideally, we want DATA, 0, [garbage], DATA. That way everything can be copied directly to GPIO with 1 DMA channel looping, with little overhead.
 *
 **Or use 1 dma channel, but additionally write to a dreq-able peripheral (PWM):
 *   By using control-blocks, one can copy a word to the GPIOs, then have the next CB copy a word to the PWM fifo, and repeat
 *   By having BOTH control-blocks be dreq-limited by the PWM's dreq, they can BOTH be rate-limited.
 *   PWM clock works as so: 500MHz / clock_div = PWM_BITRATE (note: bitrate!)
 *     PWM_BITRATE / PWM_RNG1 = #of FIFO writes/sec
 *     Max PWM_BITRATE = 25MHz
 *   Also, dest_addr = 0x7e20b000 // the testbus interface which is a dump peripheral that goes nowhere (http://www.raspberrypi.org/forums/viewtopic.php?f=37&t=7696&start=25 )
 *
 * What does the documentation mean on page 140 when it says "PWM DMA is mapped to DMA channel 5."? My guess is that's a typo and it means PWM DREQ is at index 5 in the DREQ mapping.
 *
 * DMA Control Block layout:
 *   repeat #srcBlock times:
 *     1.copy srcBlock to gpios
 *     2.zero srcBlock
 *     3.move byte to PWM (paced via DREQ)
 *   These are largely redundant; it may be possible to use less memory (each cb uses 32 bytes of memory)
 *
 * Problem: each "frame" is currently 6 words (but the last word is padding), and 1 PAGE_SIZE is not an integer multiple of 6*4
 *   Thus, the very last frame on each page cannot be used with DMA. Because of this, too, the virtual addressing of each frame is messed up - we must skip one frame per page.
 *   One solution is to append 2 more pad words to each frame (so that it is 8 words in length). This fixes all issues, but increases ram usage and potentially cache problems (L2 is 128KB). However, since data reads are sequential, even if all data doesn't fit in cache, it will be prefetched.
 *   Another solution is to decrease frame size to 4 words, and use 2 control blocks for each frame (thus eliminating the 1-byte padding in the center). This will have an even LARGER impact on ram usage - effectively using 20 words/frame vs current 14 words/frame & alternative 16words/frame
 *   Another solution is to directly mix src data with CB data. Each CB has 2 words of padding, and a data frame is 5 words, and each CB must be aligned to 8 words. Therefore, the following is possible, assuming each frame requires 3 CBs:
 *     CB1.1(padded) | CB1.2(padded) | CB1.3(padded) | CB2.1(padded) | CB2.2(padded) | CB2.3(unpadded) | SRC(5) | SRC(5) <- uses 56 words per 2 frames 
 *     HOWEVER, PAGE_SIZE is not an integral multiple of 56 words
 *     Although, 1 of those CBs (the one which zeros the previous source) could be shared amongst multiple frames - that is, only zero every, say, 4 frames. The effect is:
 *      *32 words for 1 frame  grouped (5  src words - 2 means pad to 8 words for src)
 *       48 words for 2 frames grouped (10 src words - 2 means pad to 8 words for src)
 *       72 words for 3 frames grouped (15 src words - 2 means pad to 16 words for src)
 *       96 words for 4 frames grouped (20 src words - 2 means pad to 24 words for src)
 *       112 words for 5 frames grouped(25 src words - 2 means pad to 24 words for src)
 *       136 words for 6 frames grouped(30 src words - 2 means pad to 32 words for src)
 *       160 words for 7 frames grouped(35 src words - 2 means pad to 40 words for src)
 *       176 words for 8 frames grouped(40 src words - 2 means pad to 40 words for src)
 *       200 words for 9 frames grouped(45 src words - 2 means pad to 48 words for src)
 *       216 words for 10frames grouped(50 src words - 2 means pad to 48 words for src)
 *       240 words for 11frames grouped(55 src words - 2 means pad to 56 words for src)
 *       264 words for 12frames grouped(60 src words - 2 means pad to 64 words for src)
 *    ...432 words for 20frames grouped(100src words - 2 means pad to 104 words for src)
 *   ...*512 words for 24frames grouped(120src words - 2 means pad to 120 words for src)
 *     As can be seen, this still requires extra padding. Could do 128 words for 5 frames, or 256 words for 11 frames (23.3 words/frame), and that requires funky math.
 *     The 24 frame option would work OK. 24 is a relatively easy number to work with, and 21.3 words/frame (limit is 21 words/frame)
 *    Another solution is to use the 2D stride functionality. The source frame is really 4 words and the destination is really 2 words, a 1 word gap, and then the other 2 words. Thus 2d stride can be used to skip over that one word gap.
 *
 *  How to determine the current source word being processed?
 *    dma header points to the physical CONBLOCK_AD. This can be linked to the virtual source address via a map.
 *    OR: STRIDE register is unused in 1D mode. Could write the src index that this block is linked to in that register. But then we can't use stride feature.
 *      Rather, we can't use the stride feature on ONE cb per frame. So, use stride on the buffer->GPIO cb, and use the stride register to indicate index on the zeros-copy and the PWM cb. Can tell which CB we're looking at based on the 2DEN flag. If we're looking at the buffer->GPIO cb, then instead look at NEXTCON_BK
 *    NOTE: if 2d stride is disabled, it appears that the DMA engine doesn't even load the STRIDE register (it's read as garbage). It may PERHAPS display the last loaded word.
 *    Note: unused fields are read as "Don't care", meaning we can't use them to store user-data.
 *
 * http://www.raspberrypi.org/forums/viewtopic.php?f=44&t=26907
 *   Says gpu halts all DMA for 16us every 500ms. To bypass, add 'disable_pvt=1' to /boot/cmdline.txt
 * http://www.raspberrypi.org/forums/viewtopic.php?f=37&t=7696&start=25
 *   Says it's possible to get access to a 250MHz clock.
 * How to make DMA more consistent (ie reduce bus contention?):
 *   disable interrupts 1 uS before any 'real' transaction, enable them afterwards
 *   Make sure dummy writes DON'T READ FROM RAM (ie, use src_ignore = 1)
 *   boot with disable_pvt=1 (prevents gpu from halting everything to adjust ram refresh rate twice per second) in /boot/cmdline.txt. Does this affect system stability?
 */



#ifndef PLATFORMS_RPI_HARDWARESCHEDULER_H
#define PLATFORMS_RPI_HARDWARESCHEDULER_H


 
#include <stdint.h> //for uint32_t
#include <cstring> //for size_t, memset
#include <chrono> //for std::chrono::microseconds

#include "platforms/auto/chronoclock.h" //for EventClockT
#include "schedulerbase.h" //for OnIdleCpuIntervalT

//config settings:
//The DMA transaction is paced through the PWM FIFO. The PWM FIFO consumes 1 word every N uS (set in clock settings). 
//  Once the fifo has fewer than PWM_FIFO_SIZE words available, it will request more data from DMA. 
//  Thus, a high buffer length will be more resistant to clock drift, but may occasionally request multiple frames in a short succession 
//  (faster than FRAME_PER_SEC) in the presence of bus contention, whereas a low buffer length 
//  will always space frames AT LEAST 1/FRAMES_PER_SEC seconds apart, but may experience clock drift.
#define PWM_FIFO_SIZE 1
//number of gpio timeslices to buffer. These are processed at 0.25~1 million/sec, depending on clock settings (below). 
//The buffer must be sufficiently large to handle kernel preemption / OS multitasking.
//Using a power-of-two is a good idea as it simplifies some of the arithmetic (modulus operations).
//Furthermore, since this buffer manages PWM as well, the amount of cpu time taken during any queuePwm() writes is proportional to the # of frames,
//  so shorter is better.
//On the other hand, the longer the buffer, the more resolution available to PWM.
//  But keep in mind that most other systems out there only have 256 levels of PWM control, so even just 512 frames is already superior.
//#define SOURCE_BUFFER_FRAMES 8192
//#define SOURCE_BUFFER_FRAMES 16384
//#define SOURCE_BUFFER_FRAMES 32768
#define SOURCE_BUFFER_FRAMES 65536

//PWM Clock runs at 500 MHz, unless overclocking
#define NOMINAL_CLOCK_FREQ 500000000
//# of bits to be used in each PWM cycle. Effectively acts as a clock divisor for us, since the PWM clock is in bits/second
#define BITS_PER_CLOCK 10 
//# to divide the NOMINAL_CLOCK_FREQ by before passing it to the PWM peripheral.
#define CLOCK_DIV 200
//gpio frames per second is a product of the nominal clock frequency divided by BITS_PER_CLOCK and divided again by CLOCK_DIV
//At 500,000 frames/sec, memory bandwidth does not appear to be an issue (jitter of -1 to +2 uS)
//attempting 1,000,000 frames/sec results in an actual 800,000 frames/sec, though with a lot of jitter.
//Note that these numbers might vary with heavy network or usb usage.
// eg at 500,000 fps, with 1MB/sec network download, jitter is -1 to +30 uS
// at 250,000 fps, with 1MB/sec network download, jitter is only -3 to +3 uS

#define FRAMES_PER_SEC NOMINAL_CLOCK_FREQ/BITS_PER_CLOCK/CLOCK_DIV
#define SEC_TO_FRAME(s) ((int64_t)(s)*FRAMES_PER_SEC)
#define USEC_TO_FRAME(u) (SEC_TO_FRAME(u)/1000000)
#define FRAME_TO_SEC(f) ((int64_t)(f)*BITS_PER_CLOCK*CLOCK_DIV/NOMINAL_CLOCK_FREQ)
#define FRAME_TO_USEC(f) FRAME_TO_SEC((int64_t)(f)*1000000)
//Do to timing variance, an event scheduled at the very front of the queue might actually end up being placed at the *end* of the queue instead, so don't place anything into the frames < MIN_SCHED_AHEAD_FRAME ahead current frame
#define MIN_SCHED_AHEAD_FRAME (SOURCE_BUFFER_FRAMES>>8)
#define MIN_SCHED_AHEAD_USEC (FRAME_TO_USEC(MIN_SCHED_AHEAD_FRAME))
//Also want to avoid placing things too deep in the queue, for the same wrap-around issue.
//Can get away with a wider dead-space because we have looser tolerance here.
#define MAX_SCHED_AHEAD_FRAME (SOURCE_BUFFER_FRAMES - (SOURCE_BUFFER_FRAMES>>6))
#define MAX_SCHED_AHEAD_USEC (FRAME_TO_USEC(MAX_SCHED_AHEAD_FRAME))

//forward declare class defined in outputevent.h
class OutputEvent;

namespace plat {
namespace rpi {

//forward declare class defined in platforms/rpi/primitiveiopin.h
class PrimitiveIoPin;

//forward declare classes defined in the platforms/rpi/hardwarescheduler.cpp
struct DmaChannelHeader;
struct DmaControlBlock;
struct PwmHeader;
struct GpioBufferFrame;

/*
 * Underlying class that is abstracted by the actual HardwareScheduler (defined further below).
 */
class UnwrappedHardwareScheduler {
    struct DmaMem {
        //Memory used in DMA must bypass the CPU L1 cache, so we keep a L1-cached view & an L2-cache-coherent view
        void *virtL1;
        void *virtL2Coherent;
        std::size_t numPages;
        uintptr_t *pageMap;
        uintptr_t physAddrAtByteOffset(std::size_t bytes) const;
        uintptr_t virtToPhys(void *virt) const;
        inline DmaMem() {}
        DmaMem(const UnwrappedHardwareScheduler &sched, std::size_t numBytes);
    };
    int dmaCh;
    int memfd, pagemapfd;
    static DmaChannelHeader *dmaHeader; //must be static for cleanup() function
    volatile uint32_t *dmaBaseMem, *pwmBaseMem, *timerBaseMem, *clockBaseMem;
    DmaMem srcClrMem;
    DmaMem srcMem;
    DmaMem cbMem;
    GpioBufferFrame *srcArray;
    GpioBufferFrame *srcClrArray;
    DmaControlBlock *cbArr;
    int64_t _lastTimeAtFrame0;
    EventClockT::time_point _lastDmaSyncedTime;
    public:
        UnwrappedHardwareScheduler();
        static void cleanup();
        inline EventClockT::time_point schedTime(EventClockT::time_point evtTime) const {
            return EventClockT::time_point(evtTime.time_since_epoch() - std::chrono::microseconds(MAX_SCHED_AHEAD_USEC));
        }
        void queue(const OutputEvent &evt);
        void queuePwm(const PrimitiveIoPin &pin, float ratio, EventClockT::duration maxPeriod);
        bool onIdleCpu(OnIdleCpuIntervalT interval);
    private:
        void makeMaps();
        volatile uint32_t* mapPeripheral(int addr) const; //map a physical address into our virtual address space.
        void initSrcAndControlBlocks();
        uint8_t* makeUncachedMemView(void* virtaddr, size_t bytes) const;
        uintptr_t virtToPhys(void* virt) const;
        uintptr_t virtToUncachedPhys(void *virt) const;
        void initPwm();
        void initDma();
        void syncDmaTime();
        void queue(int pin, int mode, uint64_t micros);
        void sleepUntilMicros(uint64_t micros) const;
};


/*
 * implements the HardwareScheduler interface defined in platforms/generic/hardwarescheduler.h
 *
 * It works by maintaining a circular queue of, say, 10 ms in length.
 * When it is told to toggle a pin at a specific time (via the 'queue' function), it edits this queue.
 * Meanwhile, a CPU peripheral called DMA (Direct Memory Access) is constantly each frame of this queue into the memory-mapped GPIO bank at a mostly constant rate.
 * This memory streaming happens constantly, regardless of what the CPU is doing. Logically, it's almost like an entirely separate entity from the cpu.
 * 
 * DMA timing is done by configuring the PWM module to request a sample at a given rate. Once this sample is requested, the entire DMA transaction is gated until the request is fulfilled. This allows one to copy a frame into the gpio bank and then fulfill the PWM sample request, which stalls the transaction until the PWM device requests another sample.
 *
 */
class HardwareScheduler {
    UnwrappedHardwareScheduler *_sched;
    public:
        inline HardwareScheduler() {
            //Only ever create 1 actual UnwrappedHardwareScheduler,
            //but allow any number of HardwareSchedulers that just refer to this same underlying instance.
            //Note: this is only needed because other parts of the rpi platform need access to the HardwareScheduler.
            static UnwrappedHardwareScheduler singleton;
            _sched = &singleton;
        }
        inline EventClockT::time_point schedTime(EventClockT::time_point evtTime) const {
            return _sched->schedTime(evtTime);
        }
        inline void queue(const OutputEvent &evt) {
            return _sched->queue(evt);
        }
        inline void queuePwm(const PrimitiveIoPin &pin, float ratio, EventClockT::duration maxPeriod) {
            return _sched->queuePwm(pin, ratio, maxPeriod);
        }
        inline bool onIdleCpu(OnIdleCpuIntervalT interval) {
            return _sched->onIdleCpu(interval);
        }
};

}
}
#endif
