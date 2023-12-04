// Requires
//  #include "bcm2835.c"

#include "bcm2835.h"

#ifndef GLOBAL_OK
#include "global.h"
#endif

// ============================================================================
// Raspberry Pi 4 SPI support

// Call once to configure SPI0
void rpi_spi0_init()
{

    // Start SPI operations.
    // Forces RPi SPI0 pins P1-19 (MOSI), P1-21 (MISO), P1-23 (CLK),
    // P1-24 (CE0) and P1-26 (CE1) to alternate function ALT0,
    // which enables those pins for SPI interface.
    int spi_begin_success = bcm2835_spi_begin();

    // Set SPI parameters
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);    // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);                 // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024); // <= 4 MHz for internal clock
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);    // the default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
}

// Call once to configure SPI1
void rpi_spi1_init()
{

    // Start SPI1 operations.
    int spi_begin_success = bcm2835_aux_spi_begin();

    // Set SPI parameters
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);        // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                     // Data comes in on falling edge
    bcm2835_aux_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024); // <= 4 MHz for internal clock
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);        // the default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
}

// Call once to unconfgure SPI0
void rpi_spi0_end()
{
    // Unselect chip
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);

    // End SPI operations. SPI0 pins P1-19 (MOSI), P1-21 (MISO),
    // P1-23 (CLK), P1-24 (CE0) and P1-26 (CE1) are returned to
    // their default INPUT behaviour.
    bcm2835_spi_end();
}

// Call once to unconfgure SPI1
void rpi_spi1_end()
{
    // Unselect chip
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);

    // End SPI operations. SPI0 pins P1-19 (MOSI), P1-21 (MISO),
    // P1-23 (CLK), P1-24 (CE0) and P1-26 (CE1) are returned to
    // their default INPUT behaviour.
    bcm2835_aux_spi_end();
}

void rpi_spi_select(uint32_t chip)
{
    // Select a chip
    bcm2835_spi_chipSelect(chip);
}

void rpi_spi_unselect()
{
    // Unselect chip
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
}

// Raspberry Pi 4 SPI support
// ============================================================================

// ============================================================================
// Raspberry Pi 4 GPIO support

void rpi_gpio_init()
{
    bcm2835_gpio_fsel(PIN_ARC_OK, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN_ARC_OK, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_fsel(PIN_TORCH_BREAKAWAY, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN_TORCH_BREAKAWAY, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_fsel(PIN_OHMIC_PROBE, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN_OHMIC_PROBE, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_fsel(PIN_ESTOP, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN_ESTOP, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_fsel(PIN_TORCH_ON, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(PIN_OHMIC_ENABLE, BCM2835_GPIO_FSEL_OUTP);
}

// Raspberry Pi 4 GPIO support
// ============================================================================

void rpi_init() {
    bcm2835_init();

    rpi_gpio_init();

    rpi_spi0_init();
    rpi_spi1_init();  
}

void rpi_end() {
    rpi_spi0_end();
    rpi_spi1_end();

    // Close the library, deallocating any allocated memory and closing /dev/mem
    bcm2835_close();
}