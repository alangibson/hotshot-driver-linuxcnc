import sys
import spidev
import time

# SPI Constants
SPI_BUS = int(sys.argv[1])       # Usually 0
SPI_DEVICE = int(sys.argv[2])    # Usually 0 for CE0, 1 for CE1
SPI_SPEED_HZ = 1000000  # 1 MHz is typical

# TMC5041 Register Addresses (example: you can define more)
REG_GSTAT = 0x01
REG_IFCNT = 0x02
REG_XACTUAL1 = 0x21
REG_XACTUAL2 = 0x61

def spi_read_register(spi, address):
    """Read 32-bit register from TMC5041."""
    
    addr = 0x00 | (address & 0x7F)  # MSB 0 for read
    tx = [addr, 0x00, 0x00, 0x00, 0x00]
    rx = spi.xfer2(tx)
    time.sleep(0.001)  # Allow time for the chip to prepare response

    # Second read to get actual data (due to pipeline nature of TMC5041 SPI)
    rx = spi.xfer2([addr, 0x00, 0x00, 0x00, 0x00])
    result = (rx[1] << 24) | (rx[2] << 16) | (rx[3] << 8) | rx[4]
    
    return result

def main():
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode = 0b11  # Mode 3 for TMC5041

    try:
        #print("Reading TMC5041 Registers in a Loop...\n")
        #while True:
        gstat = spi_read_register(spi, REG_GSTAT)
        ifcnt = spi_read_register(spi, REG_IFCNT)
        xactual1 = spi_read_register(spi, REG_XACTUAL1)
        xactual2 = spi_read_register(spi, REG_XACTUAL2)
        drv_stat_1 = spi_read_register(spi, 0x6F)
        drv_stat_2 = spi_read_register(spi, 0x7F)
        ramp_stat_1 = spi_read_register(spi, 0x35)
        ramp_stat_2 = spi_read_register(spi, 0x55)

        print(f"GSTAT      : 0x{gstat:08X}")
        print(f"IFCNT      : 0x{ifcnt:08X}")
        print(f"XACTUAL1   : {xactual1}")
        print(f"XACTUAL2   : {xactual2}")
        print(f"DRV_STAT_1 : {drv_stat_1:32b}")
        print(f"DRV_STAT_2 : {drv_stat_2:32b}")
        print(f"RAMP_STAT_1: {ramp_stat_1:32b}")
        print(f"RAMP_STAT_2: {ramp_stat_2:32b}")
        print("-" * 40)

        # time.sleep(0.5)

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        spi.close()

if __name__ == "__main__":
    main()
