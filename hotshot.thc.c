/**
 * Arc voltage sensor
 */

#include "stdio.h"
#include "global.h"
#include "mcp3002.h"

float hotshot_arc_voltage()
{
    return mcp3002_read_voltage(ARC_VOLT_CHANNEL, REF_5V) * ARC_VOLT_DIVISION;
}
