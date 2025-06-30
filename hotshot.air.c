/**
 * Air pressure sensor
 */

#include "global.h"
#include "mcp3002.h"

float hotshot_air_pressure() {
    return mcp3002_read_voltage(AIR_PRESSURE_CHANNEL, REF_5V);
}
