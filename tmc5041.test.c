#include "stdio.h"
#include "time.h"
#include "tmc/ic/TMC5041/TMC5041.h"
#include "TMC-EVALSYSTEM/hal/SPI.h"
#include "TMC-EVALSYSTEM/hal/HAL.h"
#include "tmc5041.h"

#include "tmc5041.c"

int main() {

    printf("Calling setup_once\n");
    setup_once();
    printf("Done calling setup_once\n");

    // Current
    //
    int32_t irun = 23;
    handleParameter(WRITE, 0, 6, &irun); // TMC5041_IHOLD_IRUN TMC5041_IRUN_MASK
    int32_t ihold = 3;
    handleParameter(WRITE, 0, 7, &ihold); // TMC5041_IHOLD_IRUN TMC5041_IHOLD_MASK
    // TODO iholddelay

    // Chopper
    //
    int32_t tbl = 0b10; 2;
    handleParameter(WRITE, 0, 162, &tbl); // TMC5041_CHOPCONF TMC5041_TBL_MASK
    int32_t chm = 0;
    handleParameter(WRITE, 0, 163, &chm); // TMC5041_CHOPCONF TMC5041_CHM_MASK
    int32_t hend = 0;
    handleParameter(WRITE, 0, 165, &hend); // TMC5041_CHOPCONF TMC5041_HEND_MASK
    int32_t hstrt = 0b100; // 0;
    handleParameter(WRITE, 0, 166, &hstrt); // TMC5041_CHOPCONF TMC5041_HSTRT_MASK
    int32_t toff = 0b0100; // 5;
    handleParameter(WRITE, 0, 167, &toff); // TMC5041_CHOPCONF TMC5041_TOFF_MASK

    // Ramp
    //
    int32_t vmax = 25600;
    handleParameter(WRITE, 0, 2, &vmax); // TMC5041_VMAX "target speed"
    int32_t amax = vmax * 2;
    handleParameter(WRITE, 0, 5, &amax); // TMC5041_AMAX
    int32_t a1 = amax * 2;
    handleParameter(WRITE, 0, 15, &a1); // TMC5041_A1
    int32_t v1 = 0;
    handleParameter(WRITE, 0, 16, &v1); // TMC5041_V1
    int32_t dmax = amax; // DMAX = AMAX
    handleParameter(WRITE, 0, 17, &dmax); // TMC5041_DMAX
    int32_t d1 = a1;
    handleParameter(WRITE, 0, 18, &d1); // TMC5041_D1
    int32_t vstart = 0;
    handleParameter(WRITE, 0, 19, &vstart);
    int32_t vstop = 10;
    handleParameter(WRITE, 0, 20, &vstop); 
    // TODO uint8_t tzerowait = 0;

    // FIXME segfaults
    // int32_t vmax2 = FIELD_SET(0x00, TMC5041_VMAX_MASK, TMC5041_VMAX_SHIFT, 32000);
    // printf("handleParameter: VMAX 2 errors=%d\n", 
    //     handleParameter(WRITE, 0, 4, &vmax2)); // TMC5041_VMAX "maximum speed"

    // Note: RAMPMODE gets set in moveTo()

    // This simulates the HAL loop
    // moveTo(0, 100000);
    while (1) {
        uint32_t tick = clock();
        printf("----------------------------------------------------------\n");

        // TMC5041_CHIP1.config->state = 1;

        tmc5041_periodicJob(&TMC5041_CHIP1, tick);
        // tmc5041_periodicJob(&TMC5041_CHIP2, tick);

        // uint32_t sg_result;
        // handleParameter(READ, 0, 206, &sg_result);
        // printf("sg_result=%d\n", sg_result);

        int32_t drv_status = tmc5041_readInt(&TMC5041_CHIP1, TMC5041_DRVSTATUS(0));
        printf("TMC5041_DRVSTATUS %d\n", drv_status);

        int32_t ramp_status = tmc5041_readInt(&TMC5041_CHIP1, TMC5041_RAMPSTAT(0));
        printf("TMC5041_RAMPSTAT %d\n", ramp_status);


        uint8_t motor = 0;
        printf("tmc5041_periodicJob: xactual=%d velocity=%d oldX=%d\n", 
            TMC5041_CHIP1.config->shadowRegister[TMC5041_XACTUAL(motor)],
            TMC5041_CHIP1.velocity[motor],
            TMC5041_CHIP1.oldX[motor]);

        printf("----------------------------------------------------------\n");
    }

    return 0;
}
