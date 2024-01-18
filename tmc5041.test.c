#include "stdio.h"
#include "time.h"
#include "tmc5041.h"
#include "tmc5041.c"
#include "tmc/ic/TMC5041/TMC5041.h"
#include "TMC-EVALSYSTEM/hal/SPI.h"
#include "TMC-EVALSYSTEM/hal/HAL.h"

int main() {

    printf("Calling setup_once\n");
    setup_once();
    printf("Done calling setup_once\n");

    // This simulates the HAL loop
    while (1) {
        uint32_t tick = clock();
        printf("Calling tmc5041_periodicJob\n");
        tmc5041_periodicJob(&tmc5041_chip_1, tick);
        tmc5041_periodicJob(&tmc5041_chip_2, tick);
        printf("Done calling tmc5041_periodicJob\n");
    }

    return 0;
}
