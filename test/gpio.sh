#! /bin/bash

# // pin 38 / TORCH_FIRE
# #define PIN_TORCH_ON 20
# // pin 37 / ARC_OK
# #define PIN_ARC_OK 26
# // pin 10 / IHS_ENABLE
# #define PIN_OHMIC_ENABLE 15
# // pin 8 / IHS_SENSE
# #define PIN_OHMIC_PROBE 14
# // pin 36 / ESTOP
# #define PIN_ESTOP 16
# // pin 32 / TORCH_FLOAT
# #define PIN_TORCH_FLOAT 12
# // pin 40 / TORCH_LASER
# #define PIN_TORCH_LASER 21

echo "TORCH_FIRE on"
raspi-gpio set 20 op pd dh
sleep 3
echo "TORCH_FIRE off"
raspi-gpio set 20 dl

echo "TORCH_LASER on"
raspi-gpio set 21 op pd dh
sleep 3
echo "TORCH_LASER off"
raspi-gpio set 21 dl

echo "IHS_ENABLE on"
raspi-gpio set 15 op pd dh
sleep 3
echo "IHS_SENSE"
raspi-gpio set 14 ip pd
raspi-gpio get 14 
echo "IHS_ENABLE off"
raspi-gpio set 15 dl

echo "ARC_OK"
raspi-gpio set 26 ip pd
raspi-gpio get 26 

echo "ESTOP"
raspi-gpio set 16 ip pd
raspi-gpio get 16 

echo "TORCH_FLOAT"
raspi-gpio set 12 ip pd
raspi-gpio get 12 
