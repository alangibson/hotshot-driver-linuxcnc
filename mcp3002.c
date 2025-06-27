#include "mcp3002.h"
#include <stdint.h>
#include <stdio.h>
#include <bcm2835.h>
#include "rpi.h"

float mcp3002_read_voltage(int chip, int channel, float ref_voltage) {
    if (channel != 0 && channel != 1) {
        fprintf(stderr, "Invalid MCP3002 channel: %d\n", channel);
        return -1.0f;
    }

    // MCP3002 command byte
    uint8_t command = (0xC0 | (channel << 5)); // 0xC0 for CH0, 0xE0 for CH1
    uint8_t tx[2] = { command, 0x00 };
    uint8_t rx[2] = { 0 };

    rpi_spi_talk(0, BCM2835_SPI_MODE0, chip, (char *)tx, 2);

    // Decode 10-bit ADC value
    uint16_t high = rx[0] & 0x03;
    uint16_t low = rx[1];
    uint16_t adc_value = (high << 8) | low;

    // Convert to voltage
    float voltage = ((float)adc_value / 1024.0f) * ref_voltage;
    return voltage;
} 