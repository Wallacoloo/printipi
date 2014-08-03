#include <chrono>
#include <stdio.h> //for printf

using namespace std;

typedef chrono::steady_clock ClockT;

long costOfGetTime() {
	auto t1 = ClockT::now();
	auto t2 = ClockT::now();
	return chrono::duration_cast<chrono::nanoseconds>(t2-t1).count();
}

long getAvgCostOfGetTime(int n=40) {
	long sum = 0;
	for (int i=0; i<n; ++i) {
		sum += costOfGetTime();
	}
	return sum/n;
}

int main(int argc, char** argv) {
	printf("system_clock period: %ld/%ld, steady: %i\n", chrono::system_clock::period::num, chrono::system_clock::period::den, chrono::system_clock::is_steady);
	printf("high_resolution_clock period: %ld/%ld, steady: %i\n", chrono::high_resolution_clock::period::num, chrono::high_resolution_clock::period::den, chrono::high_resolution_clock::is_steady);
	printf("steady_clock period: %ld/%ld, steady: %i\n", chrono::steady_clock::period::num, chrono::steady_clock::period::den, chrono::steady_clock::is_steady);
	auto t = ClockT::now();
	auto dur = t.time_since_epoch();
	printf("time since epoch: %ld\n", dur.count());
	printf("time since epoch: %ld\n", ClockT::now().time_since_epoch().count());
	printf("time since epoch: %ld\n", chrono::duration_cast<chrono::nanoseconds>(ClockT::now().time_since_epoch()).count());
	printf("time since epoch: %ld\n", chrono::duration_cast<chrono::nanoseconds>(ClockT::now().time_since_epoch()).count());
	printf("Average cost of gettime: %lu\n", getAvgCostOfGetTime());
	return 0;
}
