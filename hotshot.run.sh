set -e

./hotshot.install.sh
DISPLAY=:0 taskset -c 2,3 linuxcnc ~/hotshot-driver-linuxcnc/my-plasma/my-plasma.ini
