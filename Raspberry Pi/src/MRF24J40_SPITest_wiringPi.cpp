/**
 * Example code for using a microchip mrf24j40 module to send and receive
 * packets using plain 802.15.4
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
 */
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
// #include "mrf24j.h"
// #include <stdlib.h>

#define PIN_CS 26    // CE0
#define PIN_RESET 27
#define PIN_INTERRUPT 28
#define SPI_CHANEL 0 //CE0 (0 or 1)
#define SPI_SPEED 500000

// typedef bool boolean;
// typedef unsigned int word;
// typedef uint8_t byte;

// void printShortReg(char* name, int address, int shouldBeValue, int defaultValue);
// void printLongReg(char* name, int address, int shouldBeValue, int defaultValue);
// void showInitReg();

//Mrf24j mrf(PIN_RESET, PIN_CS, PIN_INTERRUPT, SPI_CHANEL);

int main(){
    wiringPiSetup();

    if ((wiringPiSPISetupMode(SPI_CHANEL, SPI_SPEED, 0)) < 0){
        printf("Can't open the SPI bus: %s\n", strerror(errno));
        fflush(stdout);
    }

    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, LOW);
    delay(10);  // just my gut
    digitalWrite(PIN_RESET, HIGH);
    delay(20);  // from manual

    pinMode(PIN_CS, OUTPUT);

    digitalWrite(PIN_CS, LOW);
        uint8_t dir = 0x18;
        dir = dir << 1 & 0b01111110;
        if (wiringPiSPIDataRW(SPI_CHANEL, &dir, sizeof(uint8_t)) == -1){
            // TODO: look for another way to show the error.
            printf("SPI failure: %s\n", strerror(errno));
            fflush(stdout);
        }
        uint8_t data = 9;
        if (wiringPiSPIDataRW(SPI_CHANEL, &data, sizeof(uint8_t)) == -1){
            // TODO: look for another way to show the error.
            printf("SPI failure: %s\n", strerror(errno));
            fflush(stdout);
        }

        printf("MRF_PACON22: "); printf("value: 0x%02X\n",data);
    digitalWrite(PIN_CS, HIGH);

    digitalWrite(PIN_CS, LOW);
        dir = 0x2E;
        dir = dir << 1 & 0b01111110;
        if (wiringPiSPIDataRW(SPI_CHANEL, &dir, sizeof(uint8_t)) == -1){
            // TODO: look for another way to show the error.
            printf("SPI failure: %s\n", strerror(errno));
            fflush(stdout);
        }
        data = 9;
        if (wiringPiSPIDataRW(SPI_CHANEL, &data, sizeof(uint8_t)) == -1){
            // TODO: look for another way to show the error.
            printf("SPI failure: %s\n", strerror(errno));
            fflush(stdout);
        }

        printf("MRF_TXSTBL: "); printf("value: 0x%02X\n",data);
    digitalWrite(PIN_CS, HIGH);

    return 0;
}



