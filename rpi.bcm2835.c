#include "stdio.h"
#include <sched.h>
#include <sys/mman.h>
#include "string.h"
#include "bcm2835.h"
#ifndef GLOBAL_OK
#include "global.h"
#endif

void rpi_delay(uint32_t usecs) {
    bcm2835_delayMicroseconds(usecs);
}


// ============================================================================
// Raspberry Pi 4 GPIO support

void rpi_gpio_init() {
    // Initialize the bcm2835 library
    if (!bcm2835_init()) {
        fprintf(stderr, "bcm2835_init failed. Are you running as root?\n");
        return;
    }

    printf("hotshot: Initializing Raspberry Pi ARC_OK pin\n");
    bcm2835_gpio_fsel(PIN_ARC_OK, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN_ARC_OK, BCM2835_GPIO_PUD_DOWN);

    printf("hotshot: Initializing Raspberry Pi PIN_TORCH_FLOAT pin\n");
    bcm2835_gpio_fsel(PIN_TORCH_FLOAT, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN_TORCH_FLOAT, BCM2835_GPIO_PUD_DOWN);

    printf("hotshot: Initializing Raspberry Pi OHMIC_PROBE pin\n");
    bcm2835_gpio_fsel(PIN_OHMIC_PROBE, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN_OHMIC_PROBE, BCM2835_GPIO_PUD_DOWN);

    printf("hotshot: Initializing Raspberry Pi ESTOP pin\n");
    bcm2835_gpio_fsel(PIN_ESTOP, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(PIN_ESTOP, BCM2835_GPIO_PUD_DOWN);

    printf("hotshot: Initializing Raspberry Pi TORCH_ON pin\n");
    bcm2835_gpio_fsel(PIN_TORCH_ON, BCM2835_GPIO_FSEL_OUTP);

    printf("hotshot: Initializing Raspberry Pi OHMIC_ENABLE pin\n");
    bcm2835_gpio_fsel(PIN_OHMIC_ENABLE, BCM2835_GPIO_FSEL_OUTP);
}

uint8_t rpi_gpio_read(uint8_t gpio) {
    // Set the pin as input
    bcm2835_gpio_fsel(gpio, BCM2835_GPIO_FSEL_INPT);
    // Read the level
    return bcm2835_gpio_lev(gpio);
}

void rpi_gpio_on(uint8_t gpio) {
    // Set the pin as output
    bcm2835_gpio_fsel(gpio, BCM2835_GPIO_FSEL_OUTP);
    // Set the pin high
    bcm2835_gpio_set(gpio);
}

void rpi_gpio_off(uint8_t gpio) {
    // Set the pin as output
    bcm2835_gpio_fsel(gpio, BCM2835_GPIO_FSEL_OUTP);
    // Set the pin low
    bcm2835_gpio_clr(gpio);
}

void rpi_gpio_end() {
    bcm2835_close();
}

// Raspberry Pi 4 GPIO support
// ============================================================================


// ============================================================================
// Raspberry Pi 4 SPI support

// Save reference to selected chip
static uint32_t SELECTED_CHIP = 1024;

// GPIO pin numbers for chip select pins
static const uint8_t CS_GPIO_PINS[] = {
    SPI0_CS0_GPIO,
    SPI0_CS1_GPIO,
    SPI0_CS2_GPIO,
    SPI0_CS3_GPIO
};

// Call once to configure SPI0
void rpi_spi_init()
{

    // Start SPI operations.
    // Forces RPi SPI0 pins P1-19 (MOSI), P1-21 (MISO), P1-23 (CLK),
    // P1-24 (CE0) and P1-26 (CE1) to alternate function ALT0,
    // which enables those pins for SPI interface.
    bcm2835_spi_begin();

    // Configure all CS pins as outputs and initialize them high (unselected)
    for (int i = 0; i < NUM_CS_PINS; i++) {
        bcm2835_gpio_fsel(CS_GPIO_PINS[i], BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_set(CS_GPIO_PINS[i]);  // Initialize to high (unselected)
    }

    // Set SPI parameters
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);    // The default
    // FIXME need to switch between BCM2835_SPI_MODE3 for TMC5041 
    // and BCM2835_SPI_MODE0 for MCP3002
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                 // The default
    // RPi 4 has a 250 MHz core clock. 
    // Divide by 256 to get 976 KHz.
    // Max for TMC5041 is 1 MHz. Max for MCP3002 is 3.6 MHz @ 5V reference.
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);    // the default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);

    // Start SPI1 operations.
    bcm2835_aux_spi_begin();

    // Set SPI parameters
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);        // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                     // Data comes in on falling edge
    bcm2835_aux_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256); 
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);        // the default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
}

// Call once to unconfgure SPI0
void rpi_spi_end()
{
    // End SPI operations. SPI0 pins P1-19 (MOSI), P1-21 (MISO),
    // P1-23 (CLK), P1-24 (CE0) and P1-26 (CE1) are returned to
    // their default INPUT behaviour.
    bcm2835_spi_end();

    // End SPI operations. SPI0 pins P1-19 (MOSI), P1-21 (MISO),
    // P1-23 (CLK), P1-24 (CE0) and P1-26 (CE1) are returned to
    // their default INPUT behaviour.
    bcm2835_aux_spi_end();
}

void rpi_spi_select(uint32_t chip)
{
    SELECTED_CHIP = chip;
}

void rpi_spi_unselect()
{
    SELECTED_CHIP = 1024;
}

void rpi_spi_transfernb(char* tbuf, char* rbuf, uint32_t len) {
    
    // Manually select chip using GPIO if a valid chip is selected
    if (SELECTED_CHIP < NUM_CS_PINS) {
        // Drive CS low to select
        rpi_gpio_off(CS_GPIO_PINS[SELECTED_CHIP]);
        
        // Add delay after CS assertion for TMC5041
        // TMC5041 requires minimum 10ns CS setup time
        // Using 100ns for safety margin
        rpi_delay(1); // Minimum delay we can reliably do, ~1µs which is plenty
    }

    bcm2835_spi_transfernb(tbuf, rbuf, len);
    
    // Manually unselect chip using GPIO if a valid chip was selected
    if (SELECTED_CHIP < NUM_CS_PINS) {
        // Drive CS high to unselect
        rpi_gpio_on(CS_GPIO_PINS[SELECTED_CHIP]);
        
        // Add hold time after CS de-assertion
        rpi_delay(1);
    }
}

// Raspberry Pi 4 SPI support
// ============================================================================

#define PWM_CHANNEL 0

void rpi_clock_init()
{
    // Set up GPCLK0
    //
    // gpio4 GPCLK0 ALT0
    // clock source: 1 = 19.2 MHz oscillator 
    // The integer divider may be 2-4095.
    // The fractional divider may be 0-4095.

    // Set up PWM
    //
    bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_2);
    // Set the GPIO pin to alternate function 5 (PWM0 output)
    // bcm2835_gpio_fsel(PIN_CLOCK, BCM2835_GPIO_FSEL_ALT5);
    // PWM channel 0 in ALT FUN 5 = Pin 12, GPIO 18
    bcm2835_gpio_fsel(RPI_GPIO_P1_12, BCM2835_GPIO_FSEL_ALT5);
    // PWM channel 0, markspace mode, enabled
    bcm2835_pwm_set_mode(PWM_CHANNEL, 1, 1);
    // Range is 2 to achieve a 50% duty cycle
    bcm2835_pwm_set_range(PWM_CHANNEL, 2);
    // Start PWM
    bcm2835_pwm_set_data(PWM_CHANNEL, 1);
}

/**
 * Set up RPi pins, IO, etc.
 * Can only be called once per program run or we get all
 * kinds of mysterious exceptions.
 */
// bool NEEDS_RPI_INIT = TRUE;
void rpi_init() {

    printf("hotshot: Initializing Raspberry Pi bcm2835 driver\n");
    bcm2835_init();
    printf("hotshot: Initializing Raspberry Pi bcm2835 driver complete\n");

    printf("hotshot: Initializing Raspberry Pi GPIO\n");
    rpi_gpio_init();
    printf("hotshot: Initializing Raspberry Pi GPIO complete\n");

    printf("hotshot: Initializing Raspberry Pi clock\n");
    rpi_clock_init();
    printf("hotshot: Initializing Raspberry Pi clock complete\n");

    printf("hotshot: Initializing Raspberry Pi SPI\n");
    rpi_spi_init();
    printf("hotshot: Initializing Raspberry Pi SPI complete\n");
}

void rpi_end() {
    
    printf("hotshot: Shut down Raspberry Pi SPI 0\n");
    rpi_spi_end();
    printf("hotshot: Shut down Raspberry Pi SPI 0 complete\n");

    // Close the library, deallocating any allocated memory and closing /dev/mem
    printf("hotshot: Shut down Raspberry Pi bcm2835 driver\n");
    bcm2835_close();
    printf("hotshot: Shut down Raspberry Pi bcm2835 driver complete\n");
}
