# It has three useful targets:
#
# modules
#   Actually build the modules
#
# clean
#   Cleans up files made by 'modules'
#
# install
#   Installs the modules
#
# Set BUILD_VERBOSE=1 env var to output make commands

obj-m += hotshot.o
complex-objs := bcm2835.o rpi.o tmc5041.o hotshot.lib.o hotshot.hal.o 
# include /usr/share/linuxcnc/Makefile.modinc
include Makefile.modinc

clean:
	$(Q)rm *.o *.so *.ver *.tmp *.sym hotshot.test 2>/dev/null || true

bcm2835.o:
	$(Q)gcc -Werror bcm2835.c -o bcm2835.o -c

rpi.o: bcm2835.o
	$(Q)gcc -Werror rpi.c -o rpi.o -c

tmc5041.o: rpi.o
	$(Q)gcc -Werror -I . -I /usr/include/linuxcnc -DRTAPI tmc5041.c -o tmc5041.o -c

hotshot.lib.o: tmc5041.o
	$(Q)gcc -Werror -I . hotshot.lib.c -o hotshot.lib.o -c

hotshot.hal.o: hotshot.lib.o
	$(Q)gcc -Werror -I . -I /usr/include/linuxcnc -DRTAPI hotshot.hal.c -o hotshot.hal.o -c

hotshot.o: hotshot.hal.o 
	$(Q)halcompile --preprocess hotshot.comp
	$(Q)gcc -DRTAPI -Werror -I . -I /usr/include/linuxcnc hotshot.c -o hotshot.o -c

hotshot.test: hotshot.hal.o
	$(Q)gcc -DRTAPI -Werror -I . hotshot.test.c -o hotshot.test.o -c
	$(Q)gcc -Werror -o hotshot.test $(complex-objs) hotshot.test.o

test: hotshot.test
	echo "Make sure you sudo or you will get a segmentation fault!"
	./hotshot.test
