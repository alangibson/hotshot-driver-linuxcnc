
# Testing

To directly test the board, without involving the driver, from a Raspberry Pi

```bash
cd test
./gpio.sh
python mcp3002-spi.py
```

To run driver functional tests on the board:

```bash

```











# Set up LinuxCNC dev tools in Docker image
docker build -t linuxcnc .
# and run halcompile
docker run -it --rm -v $HOME/dev/plasma/TMC-API:/TMC-API -e C_INCLUDE_PATH=/TMC-API linuxcnc halcompile --compile /hotshot/hotshot.comp


# Install LinuxCNC 2.9 dev env on Raspberry Pi
apt update
apt install -y curl build-essential gdb
curl -O https://www.linuxcnc.org/dists/bookworm/2.9-uspace/binary-arm64/linuxcnc-uspace_2.9.1_arm64.deb
curl -O https://www.linuxcnc.org/dists/bookworm/2.9-uspace/binary-arm64/linuxcnc-uspace-dev_2.9.1_arm64.deb
curl -O https://www.linuxcnc.org/dists/bookworm/2.9-uspace/binary-arm64/linuxcnc-uspace-dbgsym_2.9.1_arm64.deb
dpkg -i linuxcnc-uspace_2.9.1_arm64.deb linuxcnc-uspace-dev_2.9.1_arm64.deb linuxcnc-uspace-dbgsym_2.9.1_arm64.deb || true
apt --fix-broken -y install


# Download TMC API
git clone https://github.com/trinamic/TMC-API.git


# Install Broadcom driver
sudo apt -y install libcap2 libcap-dev
cd ~
curl -O http://www.airspayce.com/mikem/bcm2835/bcm2835-1.73.tar.gz
tar xvfz bcm2835-1.73.tar.gz
cd bcm2835-1.73
CFLAGS="-DBCM2835_HAVE_LIBCAP" ./configure
make
sudo make install
sudo setcap cap_sys_rawio+ep $(which linuxcnc)
sudo adduser $USER kmem
echo 'SUBSYSTEM=="mem", KERNEL=="mem", GROUP="kmem", MODE="0660"' | sudo tee /etc/udev/rules.d/98-mem.rules


# Install HAL realtime component
./hotshot.install.sh


# Run hotshot driver with HAL file
halrun -i my-plasma/my-plasma.ini -f my-plasma/my-plasma.hal

# Run HAL realtime component
halrun
loadrt threads name1=servo-thread period1=1000000
loadrt hotshot
addf hotshot.0      servo-thread
# Dump in all setp statements from my-plasma.hal here
start
# or start hal when you are stuck runing as root under Docker
useradd -m linuxcnc
export RTAPI_UID=`id -u linuxcnc` RTAPI_FIFO_PATH=/home/linuxcnc/.rtapi_fifo


# Run calibration function
halrun
loadrt threads name1=servo-thread period1=1000000
loadrt hotshot
addf hotshot.0.calibrate              servo-thread
setp                hotshot.0.axis-x-velocity-cmd       50.0
setp                hotshot.0.axis-x-acceleration-cmd   1500.0
setp                hotshot.0.axis-x-sgt 5
start


# Sync from laptop to Rpi
rsync -uav --delete /home/alangibson/dev/plasma linuxcnc.local:/home/alangibson/dev


# Use HAL component directly
sudo usermod -a -G gpio $USER 
sudo usermod -a -G kmem $USER # required for /dev/mem access 
sudo chmod g+w /dev/mem
halrun


# References

PID explanation:
    https://forum.linuxcnc.org/10-advanced-configuration/14483-servo-tuning-pid-and-ffn