#include "tmc/ic/TMC5041/TMC5041.h"
#include "TMC-EVALSYSTEM/hal/SPI.h"
#include "TMC-EVALSYSTEM/hal/HAL.h"
#include "tmc/ic/TMC5041/TMC5041.c"
#include "tmc5041.h"

// SPI setup
//
static void spi_init(void);
static uint8_t spi_ch1_readWrite(uint8_t data, uint8_t lastTransfer);
static uint8_t spi_ch2_readWrite(uint8_t data, uint8_t lastTransfer);
static void spi_ch1_readWriteArray(uint8_t *data, size_t length);
static void spi_ch2_readWriteArray(uint8_t *data, size_t length);
static void reset_ch1();
static void reset_ch2();
// FIXME give real pin number
static IOPinTypeDef IODummy = {.bitWeight = DUMMY_BITWEIGHT};
SPITypeDef SPI =
    {
        .ch1 =
            {
                // .periphery       = SPI1_BASE_PTR,
                .CSN = &IODummy,
                .readWrite = spi_ch1_readWrite,
                .readWriteArray = spi_ch1_readWriteArray,
                .reset = reset_ch1},
        .ch2 =
            {
                // .periphery       = SPI2_BASE_PTR,
                .CSN = &IODummy,
                .readWrite = spi_ch2_readWrite,
                .readWriteArray = spi_ch2_readWriteArray,
                .reset = reset_ch2},
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
IOPinMapTypeDef IOMap =
    {
        .init = iomap_init,
};

static void iomap_init()
{
    // TODO ?
}

static void setPinHigh(IOPinTypeDef *pin)
{
    printf("TODO set pin high: %d\n", pin->bitWeight);
}

static void setPinLow(IOPinTypeDef *pin)
{
    printf("TODO set pin low: %d\n", pin->bitWeight);
}

uint8_t readWrite(SPIChannelTypeDef *SPIChannel, uint8_t writeData, uint8_t lastTransfer)
{

    printf("readWrite: writeData=%d lastTransfer=%d\n", writeData, lastTransfer);
    printf("readWrite: SPI bitWeight=%d\n", SPIChannel->CSN->bitWeight);

    uint8_t readData = 0;

    printf("readWrite: Calling setLow\n");

    // Chip Select
    HAL.IOs->config->setLow(SPIChannel->CSN);

    printf("readWrite: Done calling setLow\n");

    // TODO send data
    // writeData
    // TODO read the data
    // readData = SPI_POPR_REG(SPIChannel->periphery);

    if (lastTransfer)
    {
        // Chip Unselect
        // reset CSN manual, falls Probleme Auftreten, dann diese Zeile unter die while Schleife
        HAL.IOs->config->setHigh(SPIChannel->CSN);
    }

    return readData;
}

#include "stdio.h"

/**
 * This function is required by the TMC-API.
 * Implementation is inspired by TMC5041_eval.c
 */
void tmc5041_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
{
    // TODO implement this
    // Maybe combine tmc5041_read_register and tmc5041_write_register?

    printf("tmc5041_readWriteArray: channel=%d\n", channel);

    for (size_t i = 0; i < length; i++)
    {
        data[i] = readWrite(&HAL.SPI->ch1, data[i], (i == (length - 1)) ? true : false);
    }
}

uint8_t spi_ch1_readWrite(uint8_t data, uint8_t lastTransfer)
{
    return readWrite(&HAL.SPI->ch1, data, lastTransfer);
}

uint8_t spi_ch2_readWrite(uint8_t data, uint8_t lastTransfer)
{
    return readWrite(&HAL.SPI->ch2, data, lastTransfer);
}

static void spi_ch1_readWriteArray(uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        data[i] = readWrite(&HAL.SPI->ch1, data[i], (i == (length - 1)) ? true : false);
    }
}

static void spi_ch2_readWriteArray(uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        data[i] = readWrite(&HAL.SPI->ch2, data[i], (i == (length - 1)) ? true : false);
    }
}

void reset_ch1()
{
    // configure SPI1 pins PORTB_PCR11(SCK), PORTB_PCR17(SDI), PORTB_PCR15(SDO), PORTB_PCR10(CSN)
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SCK);
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SDI);
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SDO);
    HAL.IOs->config->reset(SPI.ch1.CSN);

    // set SPI0 to master mode, set inactive state of chip select to HIGH, flush both FIFO buffer by clearing their counters (Tx FIFO, and Rx FIFO are enabled)
    // SPI_MCR_REG(SPI.ch1.periphery) |= SPI_MCR_CLR_RXF_MASK | SPI_MCR_CLR_TXF_MASK;
}

void reset_ch2()
{
    //	// configure SPI2 pins PORTB_PCR21(SCK), PORTB_PCR23(SDI), PORTB_PCR22(SDO), PORTC_PCR0(CSN0), PORTA_PCR0(CSN5), PORTA_PCR4(CSN2)
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SCK);
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SDI);
    HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SDO);
    HAL.IOs->config->reset(SPI.ch2.CSN);
    SPI.ch2.readWrite = spi_ch2_readWrite;

    // set SPI0 to master mode, set inactive state of chip select to HIGH, flush both FIFO buffer by clearing their counters (Tx FIFO, and Rx FIFO are enabled)
    // SPI_MCR_REG(SPI.ch2.periphery) |= SPI_MCR_CLR_RXF_MASK | SPI_MCR_CLR_TXF_MASK;
}

// Taken from TMC5041_eval.c reset()
static uint8_t tmc5041_chip_1_reset()
{

    for (uint8_t motor = 0; motor < TMC5041_MOTORS; motor++)
        if (tmc5041_readInt(&tmc5041_chip_1, TMC5041_VACTUAL(motor)) != 0)
            return 0;

    return tmc5041_reset(&tmc5041_chip_1);
}

// Taken from TMC5041_eval.c restore()
static uint8_t tmc5041_chip_1_restore()
{
    return tmc5041_restore(&tmc5041_chip_1);
}

// Taken from TMC5041_eval.c reset()
static uint8_t tmc5041_chip_2_reset()
{

    for (uint8_t motor = 0; motor < TMC5041_MOTORS; motor++)
        if (tmc5041_readInt(&tmc5041_chip_2, TMC5041_VACTUAL(motor)) != 0)
            return 0;

    return tmc5041_reset(&tmc5041_chip_2);
}

// Taken from TMC5041_eval.c restore()
static uint8_t tmc5041_chip_2_restore()
{
    return tmc5041_restore(&tmc5041_chip_2);
}

void spi_init()
{
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
    // Cpu.initClocks();
    // Cpu.initLowLevel();
    // NVIC_init();
    // EnableInterrupts;;

    // systick_init();
    // wait(100);

    IOs.init();
    // IOMap.init();
    // LEDs.init();
    // ADCs.init();
    SPI.init();
    // WLAN.init();
    // RS232.init();
    // USB.init();

    // Determine HW version
    // get_hwid();
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
}

bool setup_once()
{

    // Configure TMC-API HAL
    //
    SPITypeDef SPI = (SPITypeDef){
        .ch1 = (SPIChannelTypeDef){
            // .CSN = &(IOPinTypeDef) {
            //     .bitWeight = 0
            // }
        },
        .ch2 = (SPIChannelTypeDef){
            // .CSN = &(IOPinTypeDef) {
            //     .bitWeight = 1
            // }
        }};

    // HAL.IOs->config
    HAL.init();

    // Configure TMC-API
    //
    ConfigurationTypeDef tmc5041_chip_config_1 = (ConfigurationTypeDef){
        .reset = &tmc5041_chip_1_reset,
        .restore = &tmc5041_chip_1_restore};
    ConfigurationTypeDef tmc5041_chip_config_2 = (ConfigurationTypeDef){
        .reset = &tmc5041_chip_2_reset,
        .restore = &tmc5041_chip_2_restore};
    SPITypeDef spi = (SPITypeDef){
        .ch1 = (SPIChannelTypeDef){
            .CSN = &(IOPinTypeDef){
                .bitWeight = 0},
            .readWrite = spi_ch1_readWrite,
            .readWriteArray = spi_ch1_readWriteArray,
            .reset = reset_ch1},
        .ch2 = (SPIChannelTypeDef){.CSN = &(IOPinTypeDef){.bitWeight = 1}, .readWrite = spi_ch2_readWrite, .readWriteArray = spi_ch2_readWriteArray, .reset = reset_ch2}};
    // Channel is just motor number
    uint8_t channel_0 = 0;
    uint8_t channel_1 = 1;
    tmc5041_init(&tmc5041_chip_1, channel_0, &tmc5041_chip_config_1, tmc5041_defaultRegisterResetState);
    tmc5041_init(&tmc5041_chip_1, channel_1, &tmc5041_chip_config_1, tmc5041_defaultRegisterResetState);
    tmc5041_init(&tmc5041_chip_2, channel_0, &tmc5041_chip_config_2, tmc5041_defaultRegisterResetState);
    tmc5041_init(&tmc5041_chip_2, channel_1, &tmc5041_chip_config_2, tmc5041_defaultRegisterResetState);
}
