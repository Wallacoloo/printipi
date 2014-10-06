//the make-system assumes all source files are .cpp, so we must wrap the bcm2835 library in a .cpp file.
extern "C" {
    #include "bcm2835.c"
}
