
echo "Starting `halrun` with INI file and HAL file."
echo
echo "Power motors on with"
echo    setp hotshot.0.axis-?-enable 1
echo
echo "Enter `start` to run the system."

# TODO recompile hotshot with DEBUG flag enabled
# Replace 
#   // #define DEBUG
# with
#   #define DEBUG
# and then run ./hotshot.install.sh

# TODO recompile hotshot with debugging symbols
# # Build with debugging symbols
# gcc -g -O0 -v -da -Q -o plasma -I TMC-API TMC-API/tmc/ic/TMC5041/TMC5041.c tmcapi.c
# OR
# gcc -g -O0 -da -Q -o plasma bcm.c -lbcm2835 -I $HOME/dev/plasma/TMC-API && sudo ./plasma
# # Start debugger
# gdb plasma
# # Debug
# (gdb) run
# then attach debugger somehow

# Load halrun with ini file set
halrun -v -I -i my-plasma/my-plasma.ini -f my-plasma/my-plasma.hal
