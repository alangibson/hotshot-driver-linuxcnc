#ifndef MCP3002_H
#define MCP3002_H

#include <stdint.h>

// Reads the voltage from the specified MCP3002 channel (0 or 1) on the given SPI chip select.
// Assumes SPI bus is already configured.
// Parameters:
//   chip: SPI chip select (CS) line to use
//   channel: MCP3002 channel (0 or 1)
//   ref_voltage: Reference voltage for ADC conversion
// Returns: Voltage as a float, or negative value on error.
float mcp3002_read_voltage(int chip, int channel, float ref_voltage);

#endif // MCP3002_H 