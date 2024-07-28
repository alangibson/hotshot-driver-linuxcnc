
# To set up development env
# - Install the Remote Explorer in local VS Code
# - Add connection to darkstar.local


# Install LinuxCNC 2.9 dev tools
LINUXCNC_VERSION=2.9.2
sudo apt update
sudo apt install -y curl build-essential gdb python3-yapps
curl -O https://www.linuxcnc.org/dists/bookworm/2.9-uspace/binary-arm64/linuxcnc-uspace_${LINUXCNC_VERSION}_arm64.deb
curl -O https://www.linuxcnc.org/dists/bookworm/2.9-uspace/binary-arm64/linuxcnc-uspace-dev_${LINUXCNC_VERSION}_arm64.deb
curl -O https://www.linuxcnc.org/dists/bookworm/2.9-uspace/binary-arm64/linuxcnc-uspace-dbgsym_${LINUXCNC_VERSION}_arm64.deb
sudo dpkg -i linuxcnc-uspace_${LINUXCNC_VERSION}_arm64.deb \
    linuxcnc-uspace-dev_${LINUXCNC_VERSION}_arm64.deb \
    linuxcnc-uspace-dbgsym_${LINUXCNC_VERSION}_arm64.deb


# Enable SPI1
# Note: pins are gpio number, not pin number
echo 'dtoverlay=spi1-3cs,cs0_pin=18,cs1_pin=17,cs2_pin=16' | sudo tee -a /boot/firmware/config.txt


# Fix permissions to /dev/mem
sudo setcap cap_sys_rawio+ep $(which linuxcnc)
sudo adduser $USER kmem
echo 'SUBSYSTEM=="mem", KERNEL=="mem", GROUP="kmem", MODE="0660"' | sudo tee /etc/udev/rules.d/98-mem.rules
sudo reboot


# Install Broadcom driver
# Creates /usr/local/lib/libbcm2835.a and /usr/local/include/bcm2835.h
sudo apt -y install libcap2 libcap-dev
curl -O http://www.airspayce.com/mikem/bcm2835/bcm2835-1.75.tar.gz
tar xvfz bcm2835-1.75.tar.gz
cd bcm2835-1.75
CFLAGS="-DBCM2835_HAVE_LIBCAP" ./configure
make
sudo make install


# Download TMC-API
git clone --depth=1 https://github.com/trinamic/TMC-API.git


# Download hotshot driver repo
git clone https://github.com/alangibson/hotshot-driver-linuxcnc.git


# Install HAL realtime component
pushd hotshot-driver-linuxcnc
./install.sh


# Test loading HAL realtime component
halrun
loadrt threads name1=servo-thread period1=1000000
loadrt hotshot
addf hotshot.0      servo-thread
# Dump in all setp statements from my-plasma.hal here
start
# or start hal when you are stuck runing as root under Docker
useradd -m linuxcnc
export RTAPI_UID=`id -u linuxcnc` RTAPI_FIFO_PATH=/home/linuxcnc/.rtapi_fifo


# Run connectivity test
./install.sh  && gcc -I . bcm2835.o rpi.o tmc5041.o tmc5041.test.c -o tmc5041.test && sudo ./tmc5041.test


# Run hotshot driver with HAL file in interactive mode
halrun -I -i my-plasma/my-plasma.ini -f my-plasma/my-plasma.hal
setp hotshot.0.axis-x-position-cmd 100000
# enter 'start' to run threads
# To manually test, comment out 
#   - net x-pos-cmd
#   - net x-enable
# in my-plasma.hal, then run 
#   halcmd: setp hotshot.0.axis-x-enable 1
#   halcmd: setp hotshot.0.axis-x-position-cmd 100000
#   halcmd: start







# Build with debugging symbols
gcc -g -O0 -v -da -Q -o plasma -I TMC-API TMC-API/tmc/ic/TMC5041/TMC5041.c tmcapi.c
OR
gcc -g -O0 -da -Q -o plasma bcm.c -lbcm2835 -I $HOME/dev/plasma/TMC-API && sudo ./plasma
# Start debugger
gdb plasma
# Debug
(gdb) run

# Use HAL component directly
sudo usermod -a -G gpio $USER 
sudo usermod -a -G kmem $USER # required for /dev/mem access 
sudo chmod g+w /dev/mem
halrun


