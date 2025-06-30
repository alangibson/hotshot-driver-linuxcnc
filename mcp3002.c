#include "stdint.h"
#include "stdio.h"
#include "global.h"
#include "rpi.h"
#include "mcp3002.h"

float mcp3002_read_voltage(int channel, float ref_voltage) {
    if (channel != 0 && channel != 1) {
        fprintf(stderr, "Invalid MCP3002 channel: %d\n", channel);
        return -1.0f;
    }

    // MCP3002 command byte
    // 0xC0 for CH0, 0xE0 for CH1
    uint8_t command = (0xC0 | (channel << 5));
    uint8_t tx[2] = { command, 0x00 };
    uint8_t rx[2] = { 0 };

    // Pause 2 usec to allow charge to build up
    rpi_delay(2);

    // Assumes rpi_spi_select(chip) has already been called
    rpi_spi_transfernb(tx, rx, 2);

    // Decode 10-bit ADC value
    uint16_t high = rx[0] & 0b00000011;
    uint16_t low = rx[1];

    uint16_t adc_value = (high << 8) | low;

    // Convert to voltage with correction factor
    float voltage = ((float)adc_value / 1024.0f) * ref_voltage;

    // FIXME Why do we need this correction factor?
    //       Is the voltage divider or the reference voltage wrong?
    // Apply same correction factor as Python test
    voltage = voltage / (float) ARC_VOLT_CORRECTION;

    return voltage;
}

