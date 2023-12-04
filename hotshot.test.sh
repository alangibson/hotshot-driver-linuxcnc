#!/bin/sh -e

DEBUG_OPTS="-g -O0 -v -da -Q"

gcc $DEBUG_OPTS bcm2835.c -o bcm2835.o -c
gcc $DEBUG_OPTS rpi.c -o rpi.o -c
gcc $DEBUG_OPTS -I . tmc5041.c -o tmc5041.o -c
gcc $DEBUG_OPTS -I . hotshot.lib.c -o hotshot.lib.o -c
gcc $DEBUG_OPTS -I . hotshot.hal.c -o hotshot.hal.o -c
gcc $DEBUG_OPTS -I . hotshot.test.c -o hotshot.test.o -c
gcc $DEBUG_OPTS -o hotshot.test *.o

sudo gdb hotshot.test
# ./hotshot.test