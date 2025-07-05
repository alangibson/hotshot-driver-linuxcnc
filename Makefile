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
complex-objs := rpi.bcm2835.o mcp3002.o tmc5041.o hotshot.lib.o hotshot.joint.o hotshot.air.c hotshot.thc.c
LIBS += -lm -lgpiod /usr/local/lib/libbcm2835.a
# include /usr/share/linuxcnc/Makefile.modinc
include Makefile.modinc

clean:
	$(Q)rm -f *.o *.so *.ver *.tmp *.sym hotshot.test 2>/dev/null || true

rpi.bcm2835.o: 
	$(Q)gcc -Werror -I/usr/local/include rpi.bcm2835.c -o rpi.bcm2835.o -c

rpi.linux.o:
	$(Q)gcc -Werror -I/usr/include/gpiod rpi.linux.c -o rpi.linux.o -c

motor.tmc5041.o: # rpi.bcm2835.o
	$(Q)gcc -Werror -I . motor.tmc5041.c -o motor.tmc5041.o -c

mcp3002.o: rpi.bcm2835.o
	$(Q)gcc -Werror mcp3002.c -o mcp3002.o -c

tmc5041.o: rpi.bcm2835.o
	$(Q)gcc -Werror -I . -I /usr/include/linuxcnc -DRTAPI tmc5041.c -o tmc5041.o -c

hotshot.lib.o:
	$(Q)gcc -Wall -I . hotshot.lib.c -o hotshot.lib.o -c

hotshot.joint.o: motor.tmc5041.o hotshot.lib.o
	$(Q)gcc -Wall -I . -I /usr/include/linuxcnc -DRTAPI hotshot.joint.c -o hotshot.joint.o -c

hotshot.air.c: mcp3002.o
	$(Q)gcc -Wall -I . -I /usr/include/linuxcnc -DRTAPI hotshot.air.c -o hotshot.air.o -c

hotshot.thc.c: mcp3002.o
	$(Q)gcc -Wall -I . -I /usr/include/linuxcnc -DRTAPI hotshot.thc.c -o hotshot.thc.o -c

hotshot.o: hotshot.joint.o hotshot.air.o hotshot.thc.o
	$(Q)halcompile --preprocess hotshot.comp
	$(Q)gcc -DRTAPI -Wall -I . -I /usr/include/linuxcnc hotshot.c -o hotshot.o -c

test.mcp3002: rpi.bcm2835.o mcp3002.o
	$(Q)gcc -Werror -I . mcp3002.test.c -o mcp3002.test.o -c
	$(Q)gcc -Werror -o mcp3002.test mcp3002.o rpi.bcm2835.o mcp3002.test.o $(LIBS)
	./mcp3002.test

test.tmc5041: # rpi.bcm2835.o motor.tmc5041.o
	$(Q)gcc -Werror -I . tmc5041.test.c -o tmc5041.test.o -c
	$(Q)gcc -Werror -o tmc5041.test rpi.bcm2835.o motor.tmc5041.o tmc5041.test.o $(LIBS)
	./tmc5041.test

test.unit: hotshot.o
	$(Q)gcc -DRTAPI -Wall -I . hotshot.test.c -o hotshot.test.o -c
	$(Q)gcc -Wall -o hotshot.test hotshot.lib.o rpi.bcm2835.o motor.tmc5041.o hotshot.test.o $(LIBS)
	./hotshot.test

test.smoke:
	@echo '*********************************************'
	@echo '* Smoke tests must be run on a Raspberry Pi *'
	@echo '*********************************************'
	@cd test
	cd test; ./smoke.sh

test.functional: hotshot.o
	@echo '******************************************************************************'
	@echo '* Functional tests must be run on a Raspberry Pi with LinuxCNC-dev installed *'
	@echo '* If test segfaults, run as root                                             *'
	@echo '******************************************************************************'
	$(Q)gcc -Wall -I . rpi.test.c -o rpi.test.o -c
	$(Q)gcc -Wall -o rpi.test rpi.bcm2835.o bcm2835.o rpi.test.o $(LIBS)
	# $(Q)gcc -DRTAPI -Wall -I . hotshot.motor.test.c -o hotshot.motor.test.o -c || true
	# $(Q)gcc -Wall -o hotshot.motor.test $(complex-objs) hotshot.motor.test.o
	# ./hotshot.motor.test
	
test: test.unit test.smoke test.functional

.PHONY: test test.unit test.smoke test.functional test.mcp3002
