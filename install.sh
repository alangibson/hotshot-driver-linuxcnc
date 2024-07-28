#!/bin/bash

gcc -c -I . bcm2835.c
gcc -c -I . rpi.c
gcc -c -I . tmc5041.c
sudo C_INCLUDE_PATH="$HOME/TMC-API" \
    EXTRA_CFLAGS="-static" \
    LDFLAGS="-L$PWD -l:bcm2835.o  -l:rpi.o -l:tmc5041.o" \
    halcompile --install hotshot.comp

# mkdir -p $HOME/dev/plasma/linuxcnc/src/hal/drivers/hotshot/
# cp -R my-plasma $HOME/dev/plasma/linuxcnc/src/hal/drivers/hotshot/
