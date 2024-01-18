
// ============================================================================
// bcm2835.h

/*! This means pin HIGH, true, 3.3volts on a pin. */
#define HIGH 0x1
/*! This means pin LOW, false, 0volts on a pin. */
#define LOW 0x0

/*! Speed of the core clock core_clk */
#define BCM2835_CORE_CLK_HZ 250000000 /*!< 250 MHz */

/*! On all recent OSs, the base of the peripherals is read from a /proc file */
#define BMC2835_RPI2_DT_FILENAME "/proc/device-tree/soc/ranges"

/*! Physical addresses for various peripheral register sets
  Base Physical Address of the BCM 2835 peripheral registers
  Note this is different for the RPi2 BCM2836, where this is derived from /proc/device-tree/soc/ranges
  If /proc/device-tree/soc/ranges exists on a RPi 1 OS, it would be expected to contain the
  following numbers:
*/
/*! Peripherals block base address on RPi 1 */
#define BCM2835_PERI_BASE 0x20000000
/*! Size of the peripherals block on RPi 1 */
#define BCM2835_PERI_SIZE 0x01000000
/*! Alternate base address for RPI  2 / 3 */
#define BCM2835_RPI2_PERI_BASE 0x3F000000
/*! Alternate base address for RPI  4 */
#define BCM2835_RPI4_PERI_BASE 0xFE000000
/*! Alternate size for RPI  4 */
#define BCM2835_RPI4_PERI_SIZE 0x01800000

/*! Offsets for the bases of various peripherals within the peripherals block
  /   Base Address of the System Timer registers
*/
#define BCM2835_ST_BASE 0x3000
/*! Base Address of the Pads registers */
#define BCM2835_GPIO_PADS 0x100000
/*! Base Address of the Clock/timer registers */
#define BCM2835_CLOCK_BASE 0x101000
/*! Base Address of the GPIO registers */
#define BCM2835_GPIO_BASE 0x200000
/*! Base Address of the SPI0 registers */
#define BCM2835_SPI0_BASE 0x204000
/*! Base Address of the BSC0 registers */
#define BCM2835_BSC0_BASE 0x205000
/*! Base Address of the PWM registers */
#define BCM2835_GPIO_PWM 0x20C000
/*! Base Address of the AUX registers */
#define BCM2835_AUX_BASE 0x215000
/*! Base Address of the AUX_SPI1 registers */
#define BCM2835_SPI1_BASE 0x215080
/*! Base Address of the AUX_SPI2 registers */
#define BCM2835_SPI2_BASE 0x2150C0
/*! Base Address of the BSC1 registers */
#define BCM2835_BSC1_BASE 0x804000

/* BEB */
/*! Base address of the SMI registers */
#define BCM2835_SMI_BASE 0x600000

/* Defines for SPI
   GPIO register offsets from BCM2835_SPI0_BASE.
   Offsets into the SPI Peripheral block in bytes per 10.5 SPI Register Map
*/
#define BCM2835_SPI0_CS 0x0000   /*!< SPI Master Control and Status */
#define BCM2835_SPI0_FIFO 0x0004 /*!< SPI Master TX and RX FIFOs */
#define BCM2835_SPI0_CLK 0x0008  /*!< SPI Master Clock Divider */
#define BCM2835_SPI0_DLEN 0x000c /*!< SPI Master Data Length */
#define BCM2835_SPI0_LTOH 0x0010 /*!< SPI LOSSI mode TOH */
#define BCM2835_SPI0_DC 0x0014   /*!< SPI DMA DREQ Controls */

/* Register masks for SPI0_CS */
#define BCM2835_SPI0_CS_LEN_LONG 0x02000000 /*!< Enable Long data word in Lossi mode if DMA_LEN is set */
#define BCM2835_SPI0_CS_DMA_LEN 0x01000000  /*!< Enable DMA mode in Lossi mode */
#define BCM2835_SPI0_CS_CSPOL2 0x00800000   /*!< Chip Select 2 Polarity */
#define BCM2835_SPI0_CS_CSPOL1 0x00400000   /*!< Chip Select 1 Polarity */
#define BCM2835_SPI0_CS_CSPOL0 0x00200000   /*!< Chip Select 0 Polarity */
#define BCM2835_SPI0_CS_RXF 0x00100000      /*!< RXF - RX FIFO Full */
#define BCM2835_SPI0_CS_RXR 0x00080000      /*!< RXR RX FIFO needs Reading (full) */
#define BCM2835_SPI0_CS_TXD 0x00040000      /*!< TXD TX FIFO can accept Data */
#define BCM2835_SPI0_CS_RXD 0x00020000      /*!< RXD RX FIFO contains Data */
#define BCM2835_SPI0_CS_DONE 0x00010000     /*!< Done transfer Done */
#define BCM2835_SPI0_CS_TE_EN 0x00008000    /*!< Unused */
#define BCM2835_SPI0_CS_LMONO 0x00004000    /*!< Unused */
#define BCM2835_SPI0_CS_LEN 0x00002000      /*!< LEN LoSSI enable */
#define BCM2835_SPI0_CS_REN 0x00001000      /*!< REN Read Enable */
#define BCM2835_SPI0_CS_ADCS 0x00000800     /*!< ADCS Automatically Deassert Chip Select */
#define BCM2835_SPI0_CS_INTR 0x00000400     /*!< INTR Interrupt on RXR */
#define BCM2835_SPI0_CS_INTD 0x00000200     /*!< INTD Interrupt on Done */
#define BCM2835_SPI0_CS_DMAEN 0x00000100    /*!< DMAEN DMA Enable */
#define BCM2835_SPI0_CS_TA 0x00000080       /*!< Transfer Active */
#define BCM2835_SPI0_CS_CSPOL 0x00000040    /*!< Chip Select Polarity */
#define BCM2835_SPI0_CS_CLEAR 0x00000030    /*!< Clear FIFO Clear RX and TX */
#define BCM2835_SPI0_CS_CLEAR_RX 0x00000020 /*!< Clear FIFO Clear RX  */
#define BCM2835_SPI0_CS_CLEAR_TX 0x00000010 /*!< Clear FIFO Clear TX  */
#define BCM2835_SPI0_CS_CPOL 0x00000008     /*!< Clock Polarity */
#define BCM2835_SPI0_CS_CPHA 0x00000004     /*!< Clock Phase */
#define BCM2835_SPI0_CS_CS 0x00000003       /*!< Chip Select */

/*! \brief bcm2835SPIBitOrder SPI Bit order
  Specifies the SPI data bit ordering for bcm2835_spi_setBitOrder()
*/
#define BCM2835_SPI_BIT_ORDER_LSBFIRST 0 /*!< LSB First */
#define BCM2835_SPI_BIT_ORDER_MSBFIRST 1 /*!< MSB First */

/*! \brief SPI Data mode
  Specify the SPI data mode to be passed to bcm2835_spi_setDataMode()
*/
#define BCM2835_SPI_MODE0 0 /*!< CPOL = 0, CPHA = 0 */
#define BCM2835_SPI_MODE1 1 /*!< CPOL = 0, CPHA = 1 */
#define BCM2835_SPI_MODE2 2 /*!< CPOL = 1, CPHA = 0 */
#define BCM2835_SPI_MODE3 3 /*!< CPOL = 1, CPHA = 1 */

/*! \brief bcm2835SPIChipSelect
  Specify the SPI chip select pin(s)
*/
#define BCM2835_SPI_CS0 0     /*!< Chip Select 0 */
#define BCM2835_SPI_CS1 1     /*!< Chip Select 1 */
#define BCM2835_SPI_CS2 2     /*!< Chip Select 2 (ie pins CS1 and CS2 are asserted) */
#define BCM2835_SPI_CS_NONE 3 /*!< No CS, control it yourself */

/*! \brief bcm2835SPIClockDivider
  Specifies the divider used to generate the SPI clock from the system clock.
  Figures below give the divider, clock period and clock frequency.
  Clock divided is based on nominal core clock rate of 250MHz on RPi1 and RPi2, and 400MHz on RPi3.
  It is reported that (contrary to the documentation) any even divider may used.
  The frequencies shown for each divider have been confirmed by measurement on RPi1 and RPi2.
  The system clock frequency on RPi3 is different, so the frequency you get from a given divider will be different.
  See comments in 'SPI Pins' for information about reliable SPI speeds.
  Note: it is possible to change the core clock rate of the RPi 3 back to 250MHz, by putting
  \code
  core_freq=250
  \endcode
  in the config.txt
*/
#define BCM2835_SPI_CLOCK_DIVIDER_65536 0     /*!< 65536 = 3.814697260kHz on Rpi2, 6.1035156kHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_32768 32768 /*!< 32768 = 7.629394531kHz on Rpi2, 12.20703125kHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_16384 16384 /*!< 16384 = 15.25878906kHz on Rpi2, 24.4140625kHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_8192 8192   /*!< 8192 = 30.51757813kHz on Rpi2, 48.828125kHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_4096 4096   /*!< 4096 = 61.03515625kHz on Rpi2, 97.65625kHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_2048 2048   /*!< 2048 = 122.0703125kHz on Rpi2, 195.3125kHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_1024 1024   /*!< 1024 = 244.140625kHz on Rpi2, 390.625kHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_512 512     /*!< 512 = 488.28125kHz on Rpi2, 781.25kHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_256 256     /*!< 256 = 976.5625kHz on Rpi2, 1.5625MHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_128 128     /*!< 128 = 1.953125MHz on Rpi2, 3.125MHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_64 64       /*!< 64 = 3.90625MHz on Rpi2, 6.250MHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_32 32       /*!< 32 = 7.8125MHz on Rpi2, 12.5MHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_16 16       /*!< 16 = 15.625MHz on Rpi2, 25MHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_8 8         /*!< 8 = 31.25MHz on Rpi2, 50MHz on RPI3 */
#define BCM2835_SPI_CLOCK_DIVIDER_4 4         /*!< 4 = 62.5MHz on Rpi2, 100MHz on RPI3. Dont expect this speed to work reliably. */
#define BCM2835_SPI_CLOCK_DIVIDER_2 2         /*!< 2 = 125MHz on Rpi2, 200MHz on RPI3, fastest you can get. Dont expect this speed to work reliably.*/
#define BCM2835_SPI_CLOCK_DIVIDER_1 1         /*!< 1 = 3.814697260kHz on Rpi2, 6.1035156kHz on RPI3, same as 0/65536 */

/* Defines for AUX
  GPIO register offsets from BCM2835_AUX_BASE.
*/
#define BCM2835_AUX_IRQ 0x0000    /*!< xxx */
#define BCM2835_AUX_ENABLE 0x0004 /*!< */

#define BCM2835_AUX_ENABLE_UART1 0x01 /*!<  */
#define BCM2835_AUX_ENABLE_SPI0 0x02  /*!< SPI0 (SPI1 in the device) */
#define BCM2835_AUX_ENABLE_SPI1 0x04  /*!< SPI1 (SPI2 in the device) */

#define BCM2835_AUX_SPI_CNTL0 0x0000  /*!< */
#define BCM2835_AUX_SPI_CNTL1 0x0004  /*!< */
#define BCM2835_AUX_SPI_STAT 0x0008   /*!< */
#define BCM2835_AUX_SPI_PEEK 0x000C   /*!< Read but do not take from FF */
#define BCM2835_AUX_SPI_IO 0x0020     /*!< Write = TX, read=RX */
#define BCM2835_AUX_SPI_TXHOLD 0x0030 /*!< Write = TX keep CS, read=RX */

#define BCM2835_AUX_SPI_CLOCK_MIN 30500     /*!< 30,5kHz */
#define BCM2835_AUX_SPI_CLOCK_MAX 125000000 /*!< 125Mhz */

#define BCM2835_AUX_SPI_CNTL0_SPEED 0xFFF00000 /*!< */
#define BCM2835_AUX_SPI_CNTL0_SPEED_MAX 0xFFF  /*!< */
#define BCM2835_AUX_SPI_CNTL0_SPEED_SHIFT 20   /*!< */

#define BCM2835_AUX_SPI_CNTL0_CS0_N 0x000C0000 /*!< CS 0 low */
#define BCM2835_AUX_SPI_CNTL0_CS1_N 0x000A0000 /*!< CS 1 low */
#define BCM2835_AUX_SPI_CNTL0_CS2_N 0x00060000 /*!< CS 2 low */

#define BCM2835_AUX_SPI_CNTL0_POSTINPUT 0x00010000 /*!< */
#define BCM2835_AUX_SPI_CNTL0_VAR_CS 0x00008000    /*!< */
#define BCM2835_AUX_SPI_CNTL0_VAR_WIDTH 0x00004000 /*!< */
#define BCM2835_AUX_SPI_CNTL0_DOUTHOLD 0x00003000  /*!< */
#define BCM2835_AUX_SPI_CNTL0_ENABLE 0x00000800    /*!< */
#define BCM2835_AUX_SPI_CNTL0_CPHA_IN 0x00000400   /*!< */
#define BCM2835_AUX_SPI_CNTL0_CLEARFIFO 0x00000200 /*!< */
#define BCM2835_AUX_SPI_CNTL0_CPHA_OUT 0x00000100  /*!< */
#define BCM2835_AUX_SPI_CNTL0_CPOL 0x00000080      /*!< */
#define BCM2835_AUX_SPI_CNTL0_MSBF_OUT 0x00000040  /*!< */
#define BCM2835_AUX_SPI_CNTL0_SHIFTLEN 0x0000003F  /*!< */

#define BCM2835_AUX_SPI_CNTL1_CSHIGH 0x00000700  /*!< */
#define BCM2835_AUX_SPI_CNTL1_IDLE 0x00000080    /*!< */
#define BCM2835_AUX_SPI_CNTL1_TXEMPTY 0x00000040 /*!< */
#define BCM2835_AUX_SPI_CNTL1_MSBF_IN 0x00000002 /*!< */
#define BCM2835_AUX_SPI_CNTL1_KEEP_IN 0x00000001 /*!< */

#define BCM2835_AUX_SPI_STAT_TX_LVL 0xF0000000   /*!< */
#define BCM2835_AUX_SPI_STAT_RX_LVL 0x00F00000   /*!< */
#define BCM2835_AUX_SPI_STAT_TX_FULL 0x00000400  /*!< */
#define BCM2835_AUX_SPI_STAT_TX_EMPTY 0x00000200 /*!< */
#define BCM2835_AUX_SPI_STAT_RX_FULL 0x00000100  /*!< */
#define BCM2835_AUX_SPI_STAT_RX_EMPTY 0x00000080 /*!< */
#define BCM2835_AUX_SPI_STAT_BUSY 0x00000040     /*!< */
#define BCM2835_AUX_SPI_STAT_BITCOUNT 0x0000003F /*!< */

/* Defines for GPIO
   The BCM2835 has 54 GPIO pins.
   BCM2835 data sheet, Page 90 onwards.
*/
/*! GPIO register offsets from BCM2835_GPIO_BASE.
  Offsets into the GPIO Peripheral block in bytes per 6.1 Register View
*/
#define BCM2835_GPFSEL0 0x0000   /*!< GPIO Function Select 0 */
#define BCM2835_GPFSEL1 0x0004   /*!< GPIO Function Select 1 */
#define BCM2835_GPFSEL2 0x0008   /*!< GPIO Function Select 2 */
#define BCM2835_GPFSEL3 0x000c   /*!< GPIO Function Select 3 */
#define BCM2835_GPFSEL4 0x0010   /*!< GPIO Function Select 4 */
#define BCM2835_GPFSEL5 0x0014   /*!< GPIO Function Select 5 */
#define BCM2835_GPSET0 0x001c    /*!< GPIO Pin Output Set 0 */
#define BCM2835_GPSET1 0x0020    /*!< GPIO Pin Output Set 1 */
#define BCM2835_GPCLR0 0x0028    /*!< GPIO Pin Output Clear 0 */
#define BCM2835_GPCLR1 0x002c    /*!< GPIO Pin Output Clear 1 */
#define BCM2835_GPLEV0 0x0034    /*!< GPIO Pin Level 0 */
#define BCM2835_GPLEV1 0x0038    /*!< GPIO Pin Level 1 */
#define BCM2835_GPEDS0 0x0040    /*!< GPIO Pin Event Detect Status 0 */
#define BCM2835_GPEDS1 0x0044    /*!< GPIO Pin Event Detect Status 1 */
#define BCM2835_GPREN0 0x004c    /*!< GPIO Pin Rising Edge Detect Enable 0 */
#define BCM2835_GPREN1 0x0050    /*!< GPIO Pin Rising Edge Detect Enable 1 */
#define BCM2835_GPFEN0 0x0058    /*!< GPIO Pin Falling Edge Detect Enable 0 */
#define BCM2835_GPFEN1 0x005c    /*!< GPIO Pin Falling Edge Detect Enable 1 */
#define BCM2835_GPHEN0 0x0064    /*!< GPIO Pin High Detect Enable 0 */
#define BCM2835_GPHEN1 0x0068    /*!< GPIO Pin High Detect Enable 1 */
#define BCM2835_GPLEN0 0x0070    /*!< GPIO Pin Low Detect Enable 0 */
#define BCM2835_GPLEN1 0x0074    /*!< GPIO Pin Low Detect Enable 1 */
#define BCM2835_GPAREN0 0x007c   /*!< GPIO Pin Async. Rising Edge Detect 0 */
#define BCM2835_GPAREN1 0x0080   /*!< GPIO Pin Async. Rising Edge Detect 1 */
#define BCM2835_GPAFEN0 0x0088   /*!< GPIO Pin Async. Falling Edge Detect 0 */
#define BCM2835_GPAFEN1 0x008c   /*!< GPIO Pin Async. Falling Edge Detect 1 */
#define BCM2835_GPPUD 0x0094     /*!< GPIO Pin Pull-up/down Enable */
#define BCM2835_GPPUDCLK0 0x0098 /*!< GPIO Pin Pull-up/down Enable Clock 0 */
#define BCM2835_GPPUDCLK1 0x009c /*!< GPIO Pin Pull-up/down Enable Clock 1 */

/*!   \brief bcm2835PortFunction
  Port function select modes for bcm2835_gpio_fsel()
*/
#define BCM2835_GPIO_FSEL_INPT 0x00 /*!< Input 0b000 */
#define BCM2835_GPIO_FSEL_OUTP 0x01 /*!< Output 0b001 */
#define BCM2835_GPIO_FSEL_ALT0 0x04 /*!< Alternate function 0 0b100 */
#define BCM2835_GPIO_FSEL_ALT1 0x05 /*!< Alternate function 1 0b101 */
#define BCM2835_GPIO_FSEL_ALT2 0x06 /*!< Alternate function 2 0b110, */
#define BCM2835_GPIO_FSEL_ALT3 0x07 /*!< Alternate function 3 0b111 */
#define BCM2835_GPIO_FSEL_ALT4 0x03 /*!< Alternate function 4 0b011 */
#define BCM2835_GPIO_FSEL_ALT5 0x02 /*!< Alternate function 5 0b010 */
#define BCM2835_GPIO_FSEL_MASK 0x07 /*!< Function select bits mask 0b111 */

/*! \brief GPIO Pin Numbers

  Here we define Raspberry Pin GPIO pins on P1 in terms of the underlying BCM GPIO pin numbers.
  These can be passed as a pin number to any function requiring a pin.
  Not all pins on the RPi 26 bin IDE plug are connected to GPIO pins
  and some can adopt an alternate function.
  RPi version 2 has some slightly different pinouts, and these are values RPI_V2_*.
  RPi B+ has yet differnet pinouts and these are defined in RPI_BPLUS_*.
  At bootup, pins 8 and 10 are set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
  When SPI0 is in use (ie after bcm2835_spi_begin()), SPI0 pins are dedicated to SPI
  and cant be controlled independently.
  If you are using the RPi Compute Module, just use the GPIO number: there is no need to use one of these
  symbolic names
*/
#define RPI_GPIO_P1_03 0  /*!< Version 1, Pin P1-03 */
#define RPI_GPIO_P1_05 1  /*!< Version 1, Pin P1-05 */
#define RPI_GPIO_P1_07 4  /*!< Version 1, Pin P1-07 */
#define RPI_GPIO_P1_08 14 /*!< Version 1, Pin P1-08, defaults to alt function 0 UART0_TXD */
#define RPI_GPIO_P1_10 15 /*!< Version 1, Pin P1-10, defaults to alt function 0 UART0_RXD */
#define RPI_GPIO_P1_11 17 /*!< Version 1, Pin P1-11 */
#define RPI_GPIO_P1_12 18 /*!< Version 1, Pin P1-12, can be PWM channel 0 in ALT FUN 5 */
#define RPI_GPIO_P1_13 21 /*!< Version 1, Pin P1-13 */
#define RPI_GPIO_P1_15 22 /*!< Version 1, Pin P1-15 */
#define RPI_GPIO_P1_16 23 /*!< Version 1, Pin P1-16 */
#define RPI_GPIO_P1_18 24 /*!< Version 1, Pin P1-18 */
#define RPI_GPIO_P1_19 10 /*!< Version 1, Pin P1-19, MOSI when SPI0 in use */
#define RPI_GPIO_P1_21 9  /*!< Version 1, Pin P1-21, MISO when SPI0 in use */
#define RPI_GPIO_P1_22 25 /*!< Version 1, Pin P1-22 */
#define RPI_GPIO_P1_23 11 /*!< Version 1, Pin P1-23, CLK when SPI0 in use */
#define RPI_GPIO_P1_24 8  /*!< Version 1, Pin P1-24, CE0 when SPI0 in use */
#define RPI_GPIO_P1_26 7  /*!< Version 1, Pin P1-26, CE1 when SPI0 in use */
/* RPi Version 2 */
#define RPI_V2_GPIO_P1_03 2  /*!< Version 2, Pin P1-03 */
#define RPI_V2_GPIO_P1_05 3  /*!< Version 2, Pin P1-05 */
#define RPI_V2_GPIO_P1_07 4  /*!< Version 2, Pin P1-07 */
#define RPI_V2_GPIO_P1_08 14 /*!< Version 2, Pin P1-08, defaults to alt function 0 UART0_TXD */
#define RPI_V2_GPIO_P1_10 15 /*!< Version 2, Pin P1-10, defaults to alt function 0 UART0_RXD */
#define RPI_V2_GPIO_P1_11 17 /*!< Version 2, Pin P1-11 */
#define RPI_V2_GPIO_P1_12 18 /*!< Version 2, Pin P1-12, can be PWM channel 0 in ALT FUN 5 */
#define RPI_V2_GPIO_P1_13 27 /*!< Version 2, Pin P1-13 */
#define RPI_V2_GPIO_P1_15 22 /*!< Version 2, Pin P1-15 */
#define RPI_V2_GPIO_P1_16 23 /*!< Version 2, Pin P1-16 */
#define RPI_V2_GPIO_P1_18 24 /*!< Version 2, Pin P1-18 */
#define RPI_V2_GPIO_P1_19 10 /*!< Version 2, Pin P1-19, MOSI when SPI0 in use */
#define RPI_V2_GPIO_P1_21 9  /*!< Version 2, Pin P1-21, MISO when SPI0 in use */
#define RPI_V2_GPIO_P1_22 25 /*!< Version 2, Pin P1-22 */
#define RPI_V2_GPIO_P1_23 11 /*!< Version 2, Pin P1-23, CLK when SPI0 in use */
#define RPI_V2_GPIO_P1_24 8  /*!< Version 2, Pin P1-24, CE0 when SPI0 in use */
#define RPI_V2_GPIO_P1_26 7  /*!< Version 2, Pin P1-26, CE1 when SPI0 in use */
#define RPI_V2_GPIO_P1_29 5  /*!< Version 2, Pin P1-29 */
#define RPI_V2_GPIO_P1_31 6  /*!< Version 2, Pin P1-31 */
#define RPI_V2_GPIO_P1_32 12 /*!< Version 2, Pin P1-32 */
#define RPI_V2_GPIO_P1_33 13 /*!< Version 2, Pin P1-33 */
#define RPI_V2_GPIO_P1_35 19 /*!< Version 2, Pin P1-35, can be PWM channel 1 in ALT FUN 5  */
#define RPI_V2_GPIO_P1_36 16 /*!< Version 2, Pin P1-36 */
#define RPI_V2_GPIO_P1_37 26 /*!< Version 2, Pin P1-37 */
#define RPI_V2_GPIO_P1_38 20 /*!< Version 2, Pin P1-38 */
#define RPI_V2_GPIO_P1_40 21 /*!< Version 2, Pin P1-40 */
/* RPi Version 2, new plug P5 */
#define RPI_V2_GPIO_P5_03 28 /*!< Version 2, Pin P5-03 */
#define RPI_V2_GPIO_P5_04 29 /*!< Version 2, Pin P5-04 */
#define RPI_V2_GPIO_P5_05 30 /*!< Version 2, Pin P5-05 */
#define RPI_V2_GPIO_P5_06 31 /*!< Version 2, Pin P5-06 */
/* RPi B+ J8 header, also RPi 2 40 pin GPIO header */
#define RPI_BPLUS_GPIO_J8_03 2  /*!< B+, Pin J8-03 */
#define RPI_BPLUS_GPIO_J8_05 3  /*!< B+, Pin J8-05 */
#define RPI_BPLUS_GPIO_J8_07 4  /*!< B+, Pin J8-07 */
#define RPI_BPLUS_GPIO_J8_08 14 /*!< B+, Pin J8-08, defaults to alt function 0 UART0_TXD */
#define RPI_BPLUS_GPIO_J8_10 15 /*!< B+, Pin J8-10, defaults to alt function 0 UART0_RXD */
#define RPI_BPLUS_GPIO_J8_11 17 /*!< B+, Pin J8-11 */
#define RPI_BPLUS_GPIO_J8_12 18 /*!< B+, Pin J8-12, can be PWM channel 0 in ALT FUN 5 */
#define RPI_BPLUS_GPIO_J8_13 27 /*!< B+, Pin J8-13 */
#define RPI_BPLUS_GPIO_J8_15 22 /*!< B+, Pin J8-15 */
#define RPI_BPLUS_GPIO_J8_16 23 /*!< B+, Pin J8-16 */
#define RPI_BPLUS_GPIO_J8_18 24 /*!< B+, Pin J8-18 */
#define RPI_BPLUS_GPIO_J8_19 10 /*!< B+, Pin J8-19, MOSI when SPI0 in use */
#define RPI_BPLUS_GPIO_J8_21 9  /*!< B+, Pin J8-21, MISO when SPI0 in use */
#define RPI_BPLUS_GPIO_J8_22 25 /*!< B+, Pin J8-22 */
#define RPI_BPLUS_GPIO_J8_23 11 /*!< B+, Pin J8-23, CLK when SPI0 in use */
#define RPI_BPLUS_GPIO_J8_24 8  /*!< B+, Pin J8-24, CE0 when SPI0 in use */
#define RPI_BPLUS_GPIO_J8_26 7  /*!< B+, Pin J8-26, CE1 when SPI0 in use */
#define RPI_BPLUS_GPIO_J8_29 5  /*!< B+, Pin J8-29  */
#define RPI_BPLUS_GPIO_J8_31 6  /*!< B+, Pin J8-31  */
#define RPI_BPLUS_GPIO_J8_32 12 /*!< B+, Pin J8-32  */
#define RPI_BPLUS_GPIO_J8_33 13 /*!< B+, Pin J8-33  */
#define RPI_BPLUS_GPIO_J8_35 19 /*!< B+, Pin J8-35, can be PWM channel 1 in ALT FUN 5 */
#define RPI_BPLUS_GPIO_J8_36 16 /*!< B+, Pin J8-36  */
#define RPI_BPLUS_GPIO_J8_37 26 /*!< B+, Pin J8-37  */
#define RPI_BPLUS_GPIO_J8_38 20 /*!< B+, Pin J8-38  */
#define RPI_BPLUS_GPIO_J8_40 21 /*!< B+, Pin J8-40  */

#define BCM2835_GPIO_PUD_OFF 0x00  /*!< Off ? disable pull-up/down 0b00 */
#define BCM2835_GPIO_PUD_DOWN 0x01 /*!< Enable Pull Down control 0b01 */
#define BCM2835_GPIO_PUD_UP 0x02   /*!< Enable Pull Up control 0b10  */

/* 2711 has a different method for pin pull-up/down/enable  */
#define BCM2835_GPPUPPDN0 0x00e4 /* Pin pull-up/down for pins 15:0  */
#define BCM2835_GPPUPPDN1 0x00e8 /* Pin pull-up/down for pins 31:16 */
#define BCM2835_GPPUPPDN2 0x00ec /* Pin pull-up/down for pins 47:32 */
#define BCM2835_GPPUPPDN3 0x00f0 /* Pin pull-up/down for pins 57:48 */

/* Defines for ST
   GPIO register offsets from BCM2835_ST_BASE.
   Offsets into the ST Peripheral block in bytes per 12.1 System Timer Registers
   The System Timer peripheral provides four 32-bit timer channels and a single 64-bit free running counter.
   BCM2835_ST_CLO is the System Timer Counter Lower bits register.
   The system timer free-running counter lower register is a read-only register that returns the current value
   of the lower 32-bits of the free running counter.
   BCM2835_ST_CHI is the System Timer Counter Upper bits register.
   The system timer free-running counter upper register is a read-only register that returns the current value
   of the upper 32-bits of the free running counter.
*/
#define BCM2835_ST_CS 0x0000  /*!< System Timer Control/Status */
#define BCM2835_ST_CLO 0x0004 /*!< System Timer Counter Lower 32 bits */
#define BCM2835_ST_CHI 0x0008 /*!< System Timer Counter Upper 32 bits */

// bcm2835.h
// ============================================================================
