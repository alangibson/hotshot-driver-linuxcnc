
# Set up

This LinuxCNC component requires a properly configure realtime Linux environment. 
Make sure you followed the instructions at https://github.com/alangibson/linuxcnc-rpi-image

GPIO is configured via config.txt. If you change GPIO pins in this repo, you need to modify them in linuxcnc-rpi-image too!

# Testing

To directly test the board, without involving the driver, from a Raspberry Pi

```bash
cd test
./smoke.sh
```
