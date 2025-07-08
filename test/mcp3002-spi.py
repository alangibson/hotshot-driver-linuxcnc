import sys
import spidev

SPI_BUS = int(sys.argv[1])       # Usually 0
SPI_DEVICE = int(sys.argv[2])    # Usually 2 for CE2, 3 for CE3

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = 1000000  # 1 MHz is typical
spi.mode = 0b00

REF_VOLTAGE = 4.187
VOLTAGE_DIVISOR = 25  # Changed from 50 to correct the doubled voltage reading

# MCP3002 command byte structure (sent by master):
# Bit 7: Start bit (must be 1)
# Bit 6: SGL/DIFF (1 for single-ended on MCP3002)
# Bit 5: ODD/SIGN (Channel select for MCP3002: 0 for CH0, 1 for CH1)
# Bit 4: MSBF (Don't care for MCP3002, often set to 0)
# Bits 3-0: Don't care (often set to 0)
#
# CH0: 1000 0000 = 0x80 (differential mode)
# CH1: 1010 0000 = 0xA0 (differential mode)
channel = 0
command_byte = 0x80 if channel == 0 else 0xA0
command = [ command_byte, 0x00 ]

response = spi.xfer(command)

print('response bytes:', [hex(b) for b in response])
print('response[0] binary:', format(response[0], '08b'), 'hex:', hex(response[0]))
print('response[1] binary:', format(response[1], '08b'), 'hex:', hex(response[1]))

# Decode 10-bit ADC value
# In the MCP3002's protocol, the first byte contains the two MSBs 
# in bits 6 and 7 (not bits 0 and 1 as we're currently masking). 
# We need to shift these bits right by 6 positions after masking them.
high = (response[0] & 0b11000000) >> 6  # Get bits 7,6 and shift right
low = response[1]
adc_value = (high << 8) | low

print('adc_value', adc_value)

# Recover voltage applied to ADC
adc_volts = (adc_value / 1024.0) * REF_VOLTAGE

print('adc_volts', adc_volts)

def arc_volt():

    # TODO why is measurement wrong by this amount?
    # CORRECTION_HACK = 2.5
    # adc_volts = adc_volts / CORRECTION_HACK
    
    voltage = adc_volts * VOLTAGE_DIVISOR

    print('arc_volt', voltage)

def air_pressure():

    # min volts 0.5 V = 0 Psi
    # max volts 4.5 V = 200 Psi

    pressure_psi = (adc_volts - 0.5) * (200 / (4.5 - 0.5))
    pressure_bar = pressure_psi / 14.5038

    print('air_pressure_psi', pressure_psi)
    print('air_pressure_bar', pressure_bar)

arc_volt()
air_pressure()

