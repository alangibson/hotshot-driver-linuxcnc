#include "stdio.h"
#include <signal.h>

#include "bcm2835.h"
#include "rpi.h"

int main() {

    printf("Initializing Raspberry Pi\n\n");
    // rpi_init();
    printf("bcm2835_init\n");
    bcm2835_init();
    printf("rpi_gpio_init\n");
    rpi_gpio_init();
    printf("rpi_clock_init\n");
    rpi_clock_init();
    printf("rpi_spi0_init\n");
    rpi_spi0_init();
    printf("rpi_spi1_init\n");
    rpi_spi1_init();  

    printf("Shutting down\n");
    // rpi_end();
    printf("rpi_spi0_end\n");
    rpi_spi0_end();
    printf("rpi_spi1_end\n");
    rpi_spi1_end();
    printf("bcm2835_close\n");
    bcm2835_close();

    return 0;
}
