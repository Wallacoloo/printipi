/* Kernel modules can handle sub-microsecond interrupts: http://raspberrypi.stackexchange.com/questions/8586/arm-timer-in-kernel-module-with-precision-less-than-microsecond
cat /proc/timer_list indicates that high-res timer (hrtimer) resolution is actually 1 ns
hrtimer is available to userspace programs. http://elinux.org/High_Resolution_Timers
Or is it kernel modules? http://raspberrypi.stackexchange.com/questions/8586/arm-timer-in-kernel-module-with-precision-less-than-microsecond
hrtimers are now used in Posix timers and nanosleep and itimers: http://lwn.net/Articles/168399/
  https://www.kernel.org/doc/Documentation/timers/hrtimers.txt
*/

int main() {
	return 0;
}
