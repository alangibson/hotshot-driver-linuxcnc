#include "stdio.h"
#include "rpi.h"
#include "global.h"
#include "mcp3002.h"

int main() {
    rpi_init();

    rpi_spi_select(ARC_VOLT_CS);

    float voltage = mcp3002_read_voltage(ARC_VOLT_CHANNEL, REF_5V);
    printf("voltage: %f\n", voltage * ARC_VOLT_DIVISION);

    rpi_spi_unselect();

    rpi_end();
}