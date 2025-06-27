import spidev

BUS = 0
DEVICE = 3

spi = spidev.SpiDev()
spi.open(BUS, DEVICE)
spi.max_speed_hz = 30000
spi.mode = 0b00

# FIXME +5V_REF is actually 4.16V
REF_VOLTAGE = 5
VOLTAGE_DIVISOR = 50

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
command = [ command_byte, 0x00 ]

response = spi.xfer(command)

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

def air_pressure():

    # min volts 0.5 V = 0 Psi
    # max volts 4.5 V = 200 Psi
    min_volt, min_psi = 0.5, 0
    max_volt, max_psi = 4.5, 200

    # Calculate PSI
    pressure_psi = (adc_volts - min_volt) * (max_psi / (max_volt - min_volt))
    pressure_bar = pressure_psi / 14.5038

    print('air_pressure_psi', pressure_psi)
    print('air_pressure_bar', pressure_bar)


arc_volt()
air_pressure()