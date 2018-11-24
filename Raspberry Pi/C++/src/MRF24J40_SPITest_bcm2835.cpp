// spi.c
//
// Example program for bcm2835 library
// Shows how to interface with SPI to transfer a byte to and from an SPI device
//
// After installing bcm2835, you can build this
// with something like:
// gcc -o spi spi.c -l bcm2835
// sudo ./spi
//
// Or you can test it before installing with:
// gcc -o spi -I ../../src ../../src/bcm2835.c spi.c
// sudo ./spi
//
// Author: Mike McCauley
// Copyright (C) 2012 Mike McCauley
// $Id: RF22.h,v 1.21 2012/05/30 01:51:25 mikem Exp $

#include <bcm2835.h>
#include <wiringPi.h>
#include <stdio.h>
// #include <wiringPi.h>

#define PIN_RESET 16
#define PIN_CS 12

int main(int argc, char **argv) {
    // If you call this, it will not actually access the GPIO
    // Use for testing
    //        bcm2835_set_debug(1);

    if (!bcm2835_init()) {
        printf("bcm2835_init failed. Are you running as root??\n");
        return 1;
    }

    if (!bcm2835_spi_begin()) {
        printf("bcm2835_spi_begin failed. Are you running as root??\n");
        return 1;
    }
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);               // The default
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);  // The default
    //bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                  // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);  // the default

    // Reset mrf24j40
    bcm2835_gpio_fsel(PIN_RESET, BCM2835_GPIO_FSEL_OUTP);  // pin output
    bcm2835_gpio_write(PIN_RESET, LOW);
    bcm2835_delay(10);
    bcm2835_gpio_write(PIN_RESET, HIGH);
    bcm2835_delay(20);

    //Set PIN_CS as output
    bcm2835_gpio_fsel(PIN_CS, BCM2835_GPIO_FSEL_OUTP);  // pin output

    bcm2835_gpio_write(PIN_CS, LOW);
        uint8_t send_data = 0x18;
        send_data = send_data << 1 & 0b01111110;
        uint8_t read_data = bcm2835_spi_transfer(send_data);
        printf("Sent to SPI: 0x%X. Read back from SPI: 0x%X.\n", send_data, read_data);

        send_data = 0x00;
        read_data = bcm2835_spi_transfer(send_data);
        printf("Sent to SPI: 0x%X. Read back from SPI: 0x%X.\n\n", send_data, read_data);

    bcm2835_gpio_write(PIN_CS, HIGH);

    bcm2835_gpio_write(PIN_CS, LOW);
        send_data = 0x2E;
        send_data = send_data << 1 & 0b01111110;
        read_data = bcm2835_spi_transfer(send_data);
        printf("Sent to SPI: 0x%02X. Read back from SPI: 0x%02X.\n", send_data, read_data);

        send_data = 0x00;
        read_data = bcm2835_spi_transfer(send_data);
        printf("Sent to SPI: 0x%02X. Read back from SPI: 0x%02X.\n", send_data, read_data);
    bcm2835_gpio_write(PIN_CS, HIGH);

    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}
