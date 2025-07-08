#include "stdint.h"
#include "unistd.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "linux/spi/spidev.h"
#include "stdio.h"
#include "string.h"
#include "gpiod.h"

#define ARRAY_SIZE(array) sizeof(array) / sizeof(array[0])

// GPIO
//
// GPIO chip and line handles
static struct gpiod_chip *GPIO_CHIP = NULL;
static struct gpiod_line *GPIO_LINES[64] = {NULL}; // Support up to 64 GPIO lines

// SPI
//
// TODO move to global.h
static uint8_t MODE = SPI_MODE_3;
static uint8_t BITS = 8;
static uint32_t CLOCK = 1000000;
static uint16_t DELAY = 1;
// Save reference to selected chip
static uint32_t SELECTED_CHIP = 1024;
// File descriptors for SPI devices 0-3 on bus 0
static int SPI_HANDLES[4] = {-1, -1, -1, -1};

// ****************************************************************************
// SPI

/** Returns pointer to file handle */
int rpi_spi_open(int bus, int chip) {
    char device_path[32];

    // Build device path
    snprintf(device_path, sizeof(device_path), "/dev/spidev%d.%d", bus, chip);
    
    int fd = open(device_path, O_RDWR);
    if (fd < 0) {
        printf("Failed to open %s\n", device_path);
        return fd;
    }
    
    // Configure SPI settings
    if (ioctl(fd, SPI_IOC_WR_MODE, &MODE) < 0) {
        printf("Failed to set SPI mode for %s\n", device_path);
        close(fd);
        fd = -1;
        return fd;
    }
    
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &BITS) < 0) {
        printf("Failed to set bits per word for %s\n", device_path);
        close(fd);
        fd = -1;
        return fd;
    }
    
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &CLOCK) < 0) {
        printf("Failed to set clock speed for %s\n", device_path);
        close(fd);
        fd = -1;
        return fd;
    }
    
    printf("Successfully opened and configured %s (fd: %d)\n", device_path, fd);

    return fd;
}

void rpi_spi_init() {
    // Open SPI devices 0 through 3 on bus 0
    for (int chip = 0; chip < 4; chip++) {
        SPI_HANDLES[chip] = rpi_spi_open(0, chip);
    }
}

void rpi_spi_select(uint32_t chip) {
    // Default to spi0
    SELECTED_CHIP = chip;
}

void rpi_spi_transfernb(char* tx, char* rx, uint32_t len) {

    if (SELECTED_CHIP == 1024) {
        printf("ERROR: No chip currently selected. Call rpi_spi_select(chip) first.\n");
        return;
    }

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .delay_usecs = DELAY,
        .speed_hz = CLOCK,
        .bits_per_word = BITS,
    };

    // Get file descriptor for selected chip
    int fd = SPI_HANDLES[SELECTED_CHIP];
    if (fd < 0) {
        printf("ERROR: Could not find open SPI file handle for chipm %d\n", SELECTED_CHIP);
        return;
    }

    // Do transaction
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("IO Error. Failed to send message over SPI.");
    }

}

void rpi_spi_unselect() {
    SELECTED_CHIP = 1024;
}

void rpi_spi_close(int fd) {
    // Close SPI channel
    close(fd);
}

void rpi_spi_send(int fd, char * tx, char * rx, int length) {
    // TODO Send tx and capture rx
}

void rpi_spi_end() {
    // Close all open SPI file descriptors
    for (int i = 0; i < 4; i++) {
        if (SPI_HANDLES[i] >= 0) {
            close(SPI_HANDLES[i]);
            SPI_HANDLES[i] = -1;
        }
    }
}

// SPI
// ****************************************************************************

// ****************************************************************************
// GPIO

void rpi_gpio_init() {
    // Open GPIO chip
    GPIO_CHIP = gpiod_chip_open("/dev/gpiochip0");
    if (!GPIO_CHIP) {
        perror("Failed to open GPIO chip");
        return;
    }
}

uint8_t rpi_gpio_read(uint8_t gpio) {
    // Get the GPIO line if we haven't already
    if (!GPIO_LINES[gpio]) {
        GPIO_LINES[gpio] = gpiod_chip_get_line(GPIO_CHIP, gpio);
        if (!GPIO_LINES[gpio]) {
            fprintf(stderr, "Failed to get GPIO line %d\n", gpio);
            return 0;
        }
        
        // Request line as input
        if (gpiod_line_request_input(GPIO_LINES[gpio], "hotshot-driver") < 0) {
            fprintf(stderr, "Failed to request GPIO line %d as input\n", gpio);
            GPIO_LINES[gpio] = NULL;
            return 0;
        }
    }
    
    // Read the value
    int val = gpiod_line_get_value(GPIO_LINES[gpio]);
    if (val < 0) {
        fprintf(stderr, "Failed to read GPIO line %d\n", gpio);
        return 0;
    }
    
    return (uint8_t)val;
}

void rpi_gpio_on(uint8_t gpio) {
    // Get the GPIO line if we haven't already
    if (!GPIO_LINES[gpio]) {
        GPIO_LINES[gpio] = gpiod_chip_get_line(GPIO_CHIP, gpio);
        if (!GPIO_LINES[gpio]) {
            fprintf(stderr, "Failed to get GPIO line %d\n", gpio);
            return;
        }
        
        // Request line as output
        if (gpiod_line_request_output(GPIO_LINES[gpio], "hotshot-driver", 0) < 0) {
            fprintf(stderr, "Failed to request GPIO line %d as output\n", gpio);
            GPIO_LINES[gpio] = NULL;
            return;
        }
    }
    
    // Set the value to 1 (on)
    if (gpiod_line_set_value(GPIO_LINES[gpio], 1) < 0) {
        fprintf(stderr, "Failed to set GPIO line %d to ON\n", gpio);
    }
}

void rpi_gpio_off(uint8_t gpio) {
    // Get the GPIO line if we haven't already
    if (!GPIO_LINES[gpio]) {
        GPIO_LINES[gpio] = gpiod_chip_get_line(GPIO_CHIP, gpio);
        if (!GPIO_LINES[gpio]) {
            fprintf(stderr, "Failed to get GPIO line %d\n", gpio);
            return;
        }
        
        // Request line as output
        if (gpiod_line_request_output(GPIO_LINES[gpio], "hotshot-driver", 0) < 0) {
            fprintf(stderr, "Failed to request GPIO line %d as output\n", gpio);
            GPIO_LINES[gpio] = NULL;
            return;
        }
    }
    
    // Set the value to 0 (off)
    if (gpiod_line_set_value(GPIO_LINES[gpio], 0) < 0) {
        fprintf(stderr, "Failed to set GPIO line %d to OFF\n", gpio);
    }
}

void rpi_gpio_end() {
    // Release all GPIO lines
    for (int i = 0; i < 64; i++) {
        if (GPIO_LINES[i]) {
            gpiod_line_release(GPIO_LINES[i]);
            GPIO_LINES[i] = NULL;
        }
    }
    
    // Close GPIO chip
    if (GPIO_CHIP) {
        gpiod_chip_close(GPIO_CHIP);
        GPIO_CHIP = NULL;
    }
}

// GPIO
// ****************************************************************************

// ****************************************************************************
// PWM

void rpi_clock_init() { }

void rpi_clock_end() { }

// GPIO
// ****************************************************************************

/** Initialize Rpi resouces */
void rpi_init() {
    rpi_gpio_init();
    rpi_spi_init();
    rpi_clock_init();
}

/** Release Rpi resouces */
void rpi_end() {
    rpi_gpio_end();
    rpi_spi_end();
    rpi_clock_end();
}
