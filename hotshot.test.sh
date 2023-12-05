#!/bin/sh -e

# rm *.o || true

export BUILD_VERBOSE=1 

sudo make clean
sudo make test

# gcc $DEBUG_OPTS -Werror bcm2835.c -o bcm2835.o -c
# gcc $DEBUG_OPTS -Werror rpi.c -o rpi.o -c
# gcc $DEBUG_OPTS -Werror -I . tmc5041.c -o tmc5041.o -c
# gcc $DEBUG_OPTS -Werror -I . hotshot.lib.c -o hotshot.lib.o -c
# gcc $DEBUG_OPTS -Werror -I . hotshot.hal.c -o hotshot.hal.o -c
# gcc $DEBUG_OPTS -Werror -I . hotshot.test.c -o hotshot.test.o -c

# DEBUG_OPTS="-g -O0 -v -da -Q"
# gcc $DEBUG_OPTS bcm2835.c -o bcm2835.o -c
# gcc $DEBUG_OPTS rpi.c -o rpi.o -c
# gcc $DEBUG_OPTS -I . tmc5041.c -o tmc5041.o -c
# gcc $DEBUG_OPTS -I . hotshot.lib.c -o hotshot.lib.o -c
# gcc $DEBUG_OPTS -I . hotshot.hal.c -o hotshot.hal.o -c

# gcc $DEBUG_OPTS -I . hotshot.test.c -o hotshot.test.o -c

# gcc $DEBUG_OPTS -o hotshot.test *.o

# sudo gdb hotshot.test

# echo "Make sure you sudo or you will get a segmentation fault!"
# ./hotshot.test

# rm *.c.* || true
