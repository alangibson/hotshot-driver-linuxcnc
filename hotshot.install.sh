#!/bin/bash

# sudo C_INCLUDE_PATH="$HOME/dev/plasma/TMC-API" EXTRA_CFLAGS="-static -lbcm2835" halcompile --install hotshot.comp

BUILD_VERBOSE=1 make clean
BUILD_VERBOSE=1 sudo make install

# mkdir -p $HOME/dev/plasma/linuxcnc/src/hal/drivers/hotshot/
# cp -R my-plasma $HOME/dev/plasma/linuxcnc/src/hal/drivers/hotshot/
