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
LIBS += -lm
# include /usr/share/linuxcnc/Makefile.modinc
include Makefile.modinc

clean:
	$(Q)rm -f *.o *.so *.ver *.tmp *.sym hotshot.test 2>/dev/null || true

bcm2835.o:
	$(Q)gcc -Werror bcm2835.c -o bcm2835.o -c

rpi.o: bcm2835.o
	$(Q)gcc -Werror rpi.c -o rpi.o -c

tmc5041.o: rpi.o
	$(Q)gcc -Werror -I . -I /usr/include/linuxcnc -DRTAPI tmc5041.c -o tmc5041.o -c

hotshot.lib.o: bcm2835.o tmc5041.o
	$(Q)gcc -Wall -I . hotshot.lib.c -o hotshot.lib.o -c

hotshot.hal.o: hotshot.lib.o
	$(Q)gcc -Wall -I . -I /usr/include/linuxcnc -DRTAPI hotshot.hal.c -o hotshot.hal.o -c

hotshot.o: hotshot.hal.o 
	$(Q)halcompile --preprocess hotshot.comp
	$(Q)gcc -DRTAPI -Wall -I . -I /usr/include/linuxcnc hotshot.c -o hotshot.o -c

test.unit: hotshot.lib.o 
	$(Q)gcc -DRTAPI -Wall -I . hotshot.test.c -o hotshot.test.o -c
	$(Q)gcc -Wall -o hotshot.test hotshot.lib.o rpi.o bcm2835.o tmc5041.o hotshot.test.o $(LIBS)
	./hotshot.test

test.smoke:
	@echo '*********************************************'
	@echo '* Smoke tests must be run on a Raspberry Pi *'
	@echo '*********************************************'
	./test/gpio.sh
	python ./test/mcp3020-spi.py

test.functional: hotshot.hal.o
	@echo '******************************************************************************'
	@echo '* Functional tests must be run on a Raspberry Pi with LinuxCNC-dev installed *'
	@echo '******************************************************************************'
	$(Q)gcc -Wall -I . rpi.test.c -o rpi.test.o -c
	$(Q)gcc -Wall -o rpi.test rpi.o bcm2835.o rpi.test.o $(LIBS)
	./rpi.test
	$(Q)gcc -DRTAPI -Wall -I . hotshot.motor.test.c -o hotshot.motor.test.o -c || true
	$(Q)gcc -Wall -o hotshot.motor.test $(complex-objs) hotshot.motor.test.o
	./hotshot.motor.test
	
test: test.unit test.smoke test.functional
