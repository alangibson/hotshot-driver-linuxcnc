#include "stdint.h"
#include "stddef.h"

void rpi_delay(uint32_t usecs);

void rpi_init();
void rpi_end();

// The pigpio way
int rpi_spi_open(uint32_t bus, uint32_t chip);
void rpi_spi_transfer(int fd, char * tx, char * rx, int length);
void rpi_spi_close(int fd);

// The bcm2835 way
// Assumes SPI0 bus
void rpi_spi_select(uint32_t chip);
void rpi_spi_transfernb(char* tbuf, char* rbuf, uint32_t len);
void rpi_spi_unselect();

uint8_t rpi_gpio_read(uint8_t gpio);
void rpi_gpio_on(uint8_t gpio);
void rpi_gpio_off(uint8_t gpio);