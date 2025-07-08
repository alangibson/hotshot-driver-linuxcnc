"""
Run with: sudo python mcp3002-buspirate.py
"""

from pyBusPirateLite.SPI import SPI

REF_VOLTAGE = 5
VOLTAGE_DIVISOR = 50

spi = SPI(
    '/dev/ttyUSB0', 
    speed=115200, # Bus Pirate's serial port speed
)
spi.pins = SPI.PIN_POWER | SPI.PIN_CS 
spi.config = SPI.CFG_PUSH_PULL | SPI.CFG_IDLE
spi.speed = '30kHz'

# MCP3002 command byte structure (sent by master):
# Bit 7: Start bit (must be 1)
# Bit 6: SGL/DIFF (1 for single-ended on MCP3002)
# Bit 5: ODD/SIGN (Channel select for MCP3002: 0 for CH0, 1 for CH1)
# Bit 4: MSBF (Don't care for MCP3002, often set to 0)
# Bits 3-0: Don't care (often set to 0)
#
# CH0: 1100 0000 = 0xC0
# CH1: 1110 0000 = 0xE0
channel = 0
command_byte = 0xC0 if channel == 0 else 0xE0

# send two bytes and receive answer
spi.cs = True
response = spi.transfer([ command_byte, 0x00 ])
spi.cs = False

print('response', response)

# Decode 10-bit ADC value
high = response[0] & 0b00000011
low = response[1]
adc_value = (high << 8) | low

print('adc_value', adc_value)

# Recover voltage applied to ADC
adc_volts = (adc_value / 1024.0) * REF_VOLTAGE

print('adc_volts', adc_volts)

def arc_volt():

    #  5V : is=0.24437927663734116, want=0.1
    # 10V : is=0.49853372434017595, want=0.2
    # 15V : is=0.7429130009775171,  want=0.3
    # 20V : is=0.9970674486803519,  want=0.4

    # TODO why is measurement wrong by this ammount?
    CORRECTION_HACK = 2.5

    voltage = ( adc_volts / CORRECTION_HACK) * VOLTAGE_DIVISOR

    print('arc_volt', voltage)

arc_volt()
