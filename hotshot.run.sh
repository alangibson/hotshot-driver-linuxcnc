set -e

./hotshot.install.sh
DISPLAY=:0 linuxcnc -d ~/hotshot-driver-linuxcnc/my-plasma/my-plasma.ini