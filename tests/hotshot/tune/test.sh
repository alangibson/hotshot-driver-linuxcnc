#!/bin/bash -xe

# TODO change
#   $(Q)$(CC) -shared -Bsymbolic $(LDFLAGS) -Wl,--version-script,$*.ver -o $@ $^ -lm
# to 
#   $(Q)$(CC) -shared -Bsymbolic $(LDFLAGS) -o $@ $^ -lm
# in file
#   $(halcompile --print-modinc)

# sudo C_INCLUDE_PATH="$HOME/dev/plasma/TMC-API" EXTRA_CFLAGS="-static -lbcm2835 -fno-inline-small-functions" halcompile --install hotshot.comp

# List exported functions
# nm -D /usr/lib/linuxcnc/modules/hotshot.so | grep ' T '
# objdump -T /usr/lib/linuxcnc/modules/hotshot.so

# cp -R tests/hotshot $HOME/dev/plasma/linuxcnc/tests/
# pushd $HOME/dev/plasma/linuxcnc
# scripts/runtests.in -v tests/hotshot/
# popd

export C_INCLUDE_PATH="$HOME/dev/plasma/TMC-API:$HOME/dev/plasma/hotshot/hotshot-driver-linuxcnc:."
export LD_LIBRARY_PATH="/usr/lib/linuxcnc/modules"
gcc -D RTAPI -DSIM \
    -I${HEADERS} \
    -L/usr/lib/linuxcnc/modules \
    -o test test.c \
    -Wl,--unresolved-symbols=ignore-all \
    -L/usr/lib/linuxcnc/modules -l:hotshot.so \
    -l:hal_lib.so -l bcm2835 \
    -lnml -llinuxcnc -llinuxcnchal
./test
