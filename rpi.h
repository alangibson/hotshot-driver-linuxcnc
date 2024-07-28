#include "stdint.h"

// Define GPIO pin numbers
#define PIN_TORCH_ON 18
#define PIN_OHMIC_ENABLE 27
#define PIN_ARC_OK 26
#define PIN_ESTOP 22
#define PIN_OHMIC_PROBE 4
#define PIN_TORCH_BREAKAWAY 6

void rpi_setup_gpio();

void rpi_setup_spi0();
void rpi_setup_spi1();
void rpi_start_spi_conversation(uint32_t chip);
void rpi_spi_transfernb(const char *tbuf, char *rbuf, uint32_t len);
void rpi_end_spi_conversation();
void rpi_teardown_spi0();
void rpi_teardown_spi1();

