#include "stdio.h"
#include "bcm2835.h"
#include "rpi.h"
#include "tmc/ic/TMC5041/TMC5041.h"
#include "TMC-EVALSYSTEM/hal/HAL.h"
#include "TMC-EVALSYSTEM/boards/Board.h"
#include "TMC-EVALSYSTEM/hal/SPI.h"
#include "tmc/ic/TMC5041/TMC5041.c"
#include "tmc5041.h"
// #include "hotshot.h"
// #include "bcm2835.c"
// #include "TMC5041_eval.c"

// SPI setup
//
static void spi_init(void);
static uint8_t spi_chan1_readWrite(uint8_t data, uint8_t lastTransfer);
static uint8_t spi_chan2_readWrite(uint8_t data, uint8_t lastTransfer);
static void spi_chan1_readWriteArray(uint8_t *data, size_t length);
static void spi_chan2_readWriteArray(uint8_t *data, size_t length);
static void spi_chan1_reset();
static void spi_chan2_reset();
// FIXME give real pin number
static IOPinTypeDef IODummy = {.bitWeight = DUMMY_BITWEIGHT};
SPITypeDef SPI =
    {
        .ch1 =
            {
                // .periphery       = SPI1_BASE_PTR,
                .CSN = &IODummy,
                .readWrite = spi_chan1_readWrite,
                .readWriteArray = spi_chan1_readWriteArray,
                .reset = spi_chan1_reset},
        .ch2 =
            {
                // .periphery       = SPI2_BASE_PTR,
                .CSN = &IODummy,
                .readWrite = spi_chan2_readWrite,
                .readWriteArray = spi_chan2_readWriteArray,
                .reset = spi_chan2_reset},
        .init = spi_init};

// HAL setup
//
static void hal_init(void);
static void hal_reset(uint8_t ResetPeripherals);
static const IOsFunctionsTypeDef IOFunctions =
    {
        .config = &IOs,
        .pins = &IOMap,
};
const HALTypeDef HAL =
    {
        .init = hal_init,
        .reset = hal_reset,
        // .NVIC_DeInit  = NVIC_DeInit,
        .SPI = &SPI,
        // .USB          = &USB,
        // .LEDs         = &LEDs,
        // .ADCs         = &ADCs,
        .IOs = &IOFunctions,
        // .RS232        = &RS232,
        // .WLAN         = &WLAN,
        // .Timer        = &Timer,
        // .UART         = &UART
};

// IOs setup
//
static void ios_init();
// static void setPinConfiguration(IOPinTypeDef *pin);
// static void copyPinConfiguration(IOPinInitTypeDef *from, IOPinTypeDef*to);
// static void resetPinConfiguration(IOPinTypeDef *pin);
// static void setPin2Output(IOPinTypeDef *pin);
// static void setPin2Input(IOPinTypeDef *pin);
static void setPinHigh(IOPinTypeDef *pin);
static void setPinLow(IOPinTypeDef *pin);
// static void setPinState(IOPinTypeDef *pin, IO_States state);
// static IO_States getPinState(IOPinTypeDef *pin);
// static uint8_t isPinHigh(IOPinTypeDef *pin);
IOsTypeDef IOs =
    {
        .init = ios_init,
        // .set         = setPinConfiguration,
        // .reset       = resetPinConfiguration,
        // .copy        = copyPinConfiguration,
        // .toOutput    = setPin2Output,
        // .toInput     = setPin2Input,
        .setHigh = setPinHigh,
        .setLow = setPinLow,
        // .setToState  = setPinState,
        // .getState    = getPinState,
        // .isHigh      = isPinHigh
};

static void iomap_init();
IOPinMapTypeDef IOMap = {
    .init = iomap_init,
};

static void iomap_init()
{
    // TODO ?
}

static void setPinHigh(IOPinTypeDef *pin)
{
    // printf("Release SPI chip select: %d\n", pin->bit);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
}

static void setPinLow(IOPinTypeDef *pin)
{
    // printf("SPI chip select: %d\n", pin->bit);
    // bcm2835_spi_chipSelect(pin->bit);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
}

uint8_t readWrite(SPIChannelTypeDef *SPIChannel, uint8_t writeData, uint8_t lastTransfer)
{

    // printf("readWrite: Write byte=%d lastTransfer=%d\n", writeData, lastTransfer);
    // printf("readWrite: SPI bit=%d\n", SPIChannel->CSN->bit);

    uint8_t readData = 0;

    // Chip Select
    HAL.IOs->config->setLow(SPIChannel->CSN);

    readData = bcm2835_spi_transfer(writeData);

    if (lastTransfer)
    {
        // Chip Unselect
        // reset CSN manual, falls Probleme Auftreten, dann diese Zeile unter die while Schleife
        HAL.IOs->config->setHigh(SPIChannel->CSN);
    }

    // printf("readWrite: Read byte=%d\n", readData);

    return readData;
}

/**
 * This function is required by the TMC-API.
 * Implementation is inspired by TMC5041_eval.c
 */
void tmc5041_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
{
    // printf("tmc5041_readWriteArray: length=%d\n", length);

    for (size_t i = 0; i < length; i++)
    {
        // FIXME should work for any channel
        data[i] = readWrite(&HAL.SPI->ch1, data[i], (i == (length - 1)) ? true : false);
    }
}

uint8_t spi_chan1_readWrite(uint8_t data, uint8_t lastTransfer)
{
    return readWrite(&HAL.SPI->ch1, data, lastTransfer);
}

uint8_t spi_chan2_readWrite(uint8_t data, uint8_t lastTransfer)
{
    return readWrite(&HAL.SPI->ch2, data, lastTransfer);
}

static void spi_chan1_readWriteArray(uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        data[i] = readWrite(&HAL.SPI->ch1, data[i], (i == (length - 1)) ? true : false);
    }
}

static void spi_chan2_readWriteArray(uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        data[i] = readWrite(&HAL.SPI->ch2, data[i], (i == (length - 1)) ? true : false);
    }
}

void spi_chan1_reset()
{

    printf("spi_chan1_reset\n");

    // configure SPI1 pins PORTB_PCR11(SCK), PORTB_PCR17(SDI), PORTB_PCR15(SDO), PORTB_PCR10(CSN)
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SCK);
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SDI);
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SDO);
    HAL.IOs->config->reset(SPI.ch1.CSN);

    // set SPI0 to master mode, set inactive state of chip select to HIGH, flush both FIFO buffer by clearing their counters (Tx FIFO, and Rx FIFO are enabled)
    // SPI_MCR_REG(SPI.ch1.periphery) |= SPI_MCR_CLR_RXF_MASK | SPI_MCR_CLR_TXF_MASK;
}

void spi_chan2_reset()
{
    printf("spi_chan2_reset\n");

    //	// configure SPI2 pins PORTB_PCR21(SCK), PORTB_PCR23(SDI), PORTB_PCR22(SDO), PORTC_PCR0(CSN0), PORTA_PCR0(CSN5), PORTA_PCR4(CSN2)
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SCK);
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SDI);
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SDO);
    HAL.IOs->config->reset(SPI.ch2.CSN);
    SPI.ch2.readWrite = spi_chan2_readWrite;

    // set SPI0 to master mode, set inactive state of chip select to HIGH, flush both FIFO buffer by clearing their counters (Tx FIFO, and Rx FIFO are enabled)
    // SPI_MCR_REG(SPI.ch2.periphery) |= SPI_MCR_CLR_RXF_MASK | SPI_MCR_CLR_TXF_MASK;
}

// Taken from TMC5041_eval.c reset()
static uint8_t tmc5041_chip1_reset()
{
    printf("tmc5041_chip1_reset\n");

    for (uint8_t motor = 0; motor < TMC5041_MOTORS; motor++)
        if (tmc5041_readInt(&TMC5041_CHIP1, TMC5041_VACTUAL(motor)) != 0)
            return 0;

    return tmc5041_reset(&TMC5041_CHIP1);
}

// Taken from TMC5041_eval.c restore()
static uint8_t tmc5041_chip1_restore()
{
    printf("tmc5041_chip1_restore\n");

    return tmc5041_restore(&TMC5041_CHIP1);
}

// Taken from TMC5041_eval.c reset()
static uint8_t tmc5041_chip2_reset()
{
    printf("tmc5041_chip2_reset\n");

    for (uint8_t motor = 0; motor < TMC5041_MOTORS; motor++)
        if (tmc5041_readInt(&TMC5041_CHIP2, TMC5041_VACTUAL(motor)) != 0)
            return 0;

    return tmc5041_reset(&TMC5041_CHIP2);
}

// Taken from TMC5041_eval.c restore()
static uint8_t tmc5041_chip2_restore()
{
    return tmc5041_restore(&TMC5041_CHIP2);
}

void spi_init()
{
    // Initialize SPI
    rpi_setup_spi0();
    rpi_setup_spi1();
    
    // SPI0 -> EEPROM
    // -------------------------------------------------------------------------------
    // SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;  // enable clock for PORT C
    // SIM_SCGC6 |= SIM_SCGC6_SPI0_MASK;   // enable clock for SPI0
    // SIM_SCGC6 &= ~SIM_SCGC6_CRC_MASK;   // disable clock for CRC module

    // configure SPI0 pins:
    //     SCK: Port C, Pin 5
    //     SDI: Port C, Pin 6
    //     SDO: Port C, Pin 7
    //     CSN: Port C, Pin 8
    // HAL.IOs->pins->EEPROM_SCK.configuration.GPIO_Mode = GPIO_Mode_AF2;
    // HAL.IOs->pins->EEPROM_SI.configuration.GPIO_Mode = GPIO_Mode_AF2;
    // HAL.IOs->pins->EEPROM_SO.configuration.GPIO_Mode = GPIO_Mode_AF2;
    // HAL.IOs->pins->EEPROM_NCS.configuration.GPIO_Mode = GPIO_Mode_OUT;

    // HAL.IOs->config->set(&HAL.IOs->pins->EEPROM_SCK);
    // HAL.IOs->config->set(&HAL.IOs->pins->EEPROM_SI);
    // HAL.IOs->config->set(&HAL.IOs->pins->EEPROM_SO);
    // HAL.IOs->config->set(&HAL.IOs->pins->EEPROM_NCS);
    // HAL.IOs->config->setHigh(&HAL.IOs->pins->EEPROM_NCS);

    // setTMCSPIParameters(SPI0_BASE_PTR);

    // SPI1 -> ch1
    // -------------------------------------------------------------------------------
    // SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;  // enable clock for PORT B
    // SIM_SCGC6 |= SIM_SCGC6_SPI1_MASK;   // enable clock for SPI1
    // SIM_SCGC6 &= ~SIM_SCGC6_CRC_MASK;   // disable clock for CRC module

    // configure SPI1 pins:
    //     SCK: Port B, Pin 11
    //     SDI: Port B, Pin 16
    //     SDO: Port B, Pin 17
    //     CSN: Port B, Pin 10
    // HAL.IOs->pins->SPI1_SCK.configuration.GPIO_Mode = GPIO_Mode_AF2;
    // HAL.IOs->pins->SPI1_SDI.configuration.GPIO_Mode = GPIO_Mode_AF2;
    // HAL.IOs->pins->SPI1_SDO.configuration.GPIO_Mode = GPIO_Mode_AF2;
    // HAL.IOs->pins->SPI1_CSN.configuration.GPIO_Mode = GPIO_Mode_OUT;

    // HAL.IOs->config->set(&HAL.IOs->pins->SPI1_SCK);
    // HAL.IOs->config->set(&HAL.IOs->pins->SPI1_SDI);
    // HAL.IOs->config->set(&HAL.IOs->pins->SPI1_SDO);
    // HAL.IOs->config->set(&HAL.IOs->pins->SPI1_CSN);
    // HAL.IOs->config->setHigh(&HAL.IOs->pins->SPI1_CSN);

    // setTMCSPIParameters(SPI1_BASE_PTR);

    // SPI2 -> ch2
    // -------------------------------------------------------------------------------
    // SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;  // enable clocks
    // SIM_SCGC3 |= SIM_SCGC3_SPI2_MASK;                                                 // enable clock for SPI2
    // SIM_SCGC6 &= ~SIM_SCGC6_CRC_MASK;                                                 // disable clock for CRC module

    // // configure SPI2 pins:
    // //     SCK:  Port B, Pin 21
    // //     SDI:  Port B, Pin 22
    // //     SDO:  Port B, Pin 23
    // //     CSN0: Port C, Pin 0
    // //     CSN1: Port A, Pin 5
    // //     CSN2: Port C, Pin 4
    // HAL.IOs->pins->SPI2_SCK.configuration.GPIO_Mode = GPIO_Mode_AF2;
    // HAL.IOs->pins->SPI2_SDI.configuration.GPIO_Mode = GPIO_Mode_AF2;
    // HAL.IOs->pins->SPI2_SDO.configuration.GPIO_Mode = GPIO_Mode_AF2;
    // HAL.IOs->pins->SPI2_CSN0.configuration.GPIO_Mode = GPIO_Mode_OUT;
    // HAL.IOs->pins->SPI2_CSN1.configuration.GPIO_Mode = GPIO_Mode_OUT;
    // HAL.IOs->pins->SPI2_CSN2.configuration.GPIO_Mode = GPIO_Mode_OUT;

    // HAL.IOs->config->set(&HAL.IOs->pins->SPI2_SCK);
    // HAL.IOs->config->set(&HAL.IOs->pins->SPI2_SDI);
    // HAL.IOs->config->set(&HAL.IOs->pins->SPI2_SDO);
    // HAL.IOs->config->set(&HAL.IOs->pins->SPI2_CSN0);
    // HAL.IOs->config->set(&HAL.IOs->pins->SPI2_CSN1);
    // HAL.IOs->config->set(&HAL.IOs->pins->SPI2_CSN2);
    // HAL.IOs->config->setHigh(&HAL.IOs->pins->SPI2_CSN0);
    // HAL.IOs->config->setHigh(&HAL.IOs->pins->SPI2_CSN1);
    // HAL.IOs->config->setHigh(&HAL.IOs->pins->SPI2_CSN2);

    // setTMCSPIParameters(SPI2_BASE_PTR);

    // // configure default SPI channel_1
    // SPIChannel_1_default = &HAL.SPI->ch1;
    // SPIChannel_1_default->CSN = &HAL.IOs->pins->SPI1_CSN;
    // // configure default SPI channel_2
    // SPIChannel_2_default = &HAL.SPI->ch2;
    // SPIChannel_2_default->CSN = &HAL.IOs->pins->SPI2_CSN0;
}

static void hal_init(void)
{
    IOs.init();
    SPI.init();
}

static void hal_reset(uint8_t ResetPeripherals)
{
    // TODO
}

void ios_init()
{
    // Set the Clock divider for core/system clock
    // 96 MHz / 2 = 48 MHz
    // SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(1);

    // SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK| SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK |SIM_SCGC5_PORTD_MASK);

    // // Ausgabe des 16Mhz Taktes auf den CLK16 Pin
    // SIM_SOPT2 &= ~SIM_SOPT2_CLKOUTSEL_MASK;
    // SIM_SOPT2 |= SIM_SOPT2_CLKOUTSEL(6);
    // PORTC_PCR3 = PORT_PCR_MUX(5);

    rpi_setup_gpio();
}

// -------------------------------------------------------------
// Copied from hotshot.comp

// void rpi_setup_gpio()
// {
//     bcm2835_gpio_fsel(PIN_ARC_OK, BCM2835_GPIO_FSEL_INPT);
//     bcm2835_gpio_set_pud(PIN_ARC_OK, BCM2835_GPIO_PUD_DOWN);
//     bcm2835_gpio_fsel(PIN_TORCH_BREAKAWAY, BCM2835_GPIO_FSEL_INPT);
//     bcm2835_gpio_set_pud(PIN_TORCH_BREAKAWAY, BCM2835_GPIO_PUD_DOWN);
//     bcm2835_gpio_fsel(PIN_OHMIC_PROBE, BCM2835_GPIO_FSEL_INPT);
//     bcm2835_gpio_set_pud(PIN_OHMIC_PROBE, BCM2835_GPIO_PUD_DOWN);
//     bcm2835_gpio_fsel(PIN_ESTOP, BCM2835_GPIO_FSEL_INPT);
//     bcm2835_gpio_set_pud(PIN_ESTOP, BCM2835_GPIO_PUD_DOWN);
//     bcm2835_gpio_fsel(PIN_TORCH_ON, BCM2835_GPIO_FSEL_OUTP);
//     bcm2835_gpio_fsel(PIN_OHMIC_ENABLE, BCM2835_GPIO_FSEL_OUTP);
// }

// // Call once to configure SPI0
// void rpi_setup_spi0()
// {

//     // Start SPI operations.
//     // Forces RPi SPI0 pins P1-19 (MOSI), P1-21 (MISO), P1-23 (CLK),
//     // P1-24 (CE0) and P1-26 (CE1) to alternate function ALT0,
//     // which enables those pins for SPI interface.
//     int spi_begin_success = bcm2835_spi_begin();

//     // Set SPI parameters
//     bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);    // The default
//     bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);                 // The default
//     bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024); // <= 4 MHz for internal clock
//     bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);    // the default
//     bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
// }

// // Call once to configure SPI1
// void rpi_setup_spi1()
// {

//     // Start SPI1 operations.
//     int spi_begin_success = bcm2835_aux_spi_begin();

//     // Set SPI parameters
//     bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);        // The default
//     bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                     // Data comes in on falling edge
//     bcm2835_aux_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024); // <= 4 MHz for internal clock
//     bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);        // the default
//     bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
// }

// Copied from hotshot.comp
// -------------------------------------------------------------

ConfigurationTypeDef tmc5041_chip1_config = (ConfigurationTypeDef){
    .reset = &tmc5041_chip1_reset,
    .restore = &tmc5041_chip1_restore};
ConfigurationTypeDef tmc5041_chip2_config = (ConfigurationTypeDef){
    .reset = &tmc5041_chip2_reset,
    .restore = &tmc5041_chip2_restore};

bool setup_once()
{
    // Intialize Broadcom driver
    int init_success = bcm2835_init();

    // Set up RPi GPIO
    // rpi_setup_gpio();

    // Initialize SPI
    // rpi_setup_spi0();
    // rpi_setup_spi1();

    // Configure TMC-API
    // 
    // Note: this just initializes TMC5041_CHIP*. 
    // It does not write anything until tmc5041_periodicJob() is called.
    HAL.init();
    //
    // Channel is just motor number
    tmc5041_init(&TMC5041_CHIP1, 0, &tmc5041_chip1_config, tmc5041_defaultRegisterResetState);
    tmc5041_init(&TMC5041_CHIP2, 0, &tmc5041_chip2_config, tmc5041_defaultRegisterResetState);
    // Force flush config to chip on first periodicJob
    TMC5041_CHIP1.config->state = CONFIG_RESET;

    // enableDriver(1); // DriverState.DRIVER_ENABLE
}
