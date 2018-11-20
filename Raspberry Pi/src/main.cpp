
#include <mrf24j_wiringPi.h>
#include <wiringPi.h>
#include <stdio.h>

const int pin_cs = 26;
const int pin_reset = 27;
const int pin_interrupt = 28;
const int spi_channel = 0; //CE0 (0 or 1)

void testRW(int printResults, int* totalRead, int* totalSuccRead, int* totalWrite, int* totalSuccWrite);
int assertReadLong(char* name, int address, int shouldBeValue, int printResults);
int assertReadShort(char* name, int address, int shouldBeValue, int printResults);
int assertWriteShort(char* name, int address, int toWriteValue, int printResults);
int assertWriteLong(char* name, int address, int toWriteValue, int printResults);

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt, spi_channel);

int main() {
    int totalRead = 0, totalReadSucc = 0, totalWrite = 0, totalWriteSucc = 0;
    int auxTotalRead, auxTotalReadSucc, auxTotalWrite, auxTotalWriteSucc;
    printf("\n");
    printf("Realizando test de lectura y escritura...\n");
    size_t count = 100;
    int printResoults = 0;
    //unsigned int testDuration = bcm2835_st_read(); // st= system timer
    for (size_t i = 0; i < count; i++) {
        mrf.reset();
        testRW(printResoults, &auxTotalRead, &auxTotalReadSucc, &auxTotalWrite, &auxTotalWriteSucc);
        totalRead += auxTotalRead;
        totalReadSucc += auxTotalRead;
        totalWrite += auxTotalWrite;
        totalWriteSucc += auxTotalWriteSucc;
    }
    //testDuration = millis() - testDuration;

    printf("\n");
    printf("------------------Resumen------------------\n");
    printf("\n");
    printf("Lecturas exitosas: %i\n", totalReadSucc);
    printf("Lecturas fallidas: %i\n", totalRead - totalReadSucc);
    printf("\n");
    printf("Escrituras exitosas: %i\n", totalWriteSucc);
    printf("Escrituras fallidas: %i\n", totalWrite - totalWriteSucc);
    printf("\n");
    // printf("Test duration: %i ms\n", testDuration);
    // printf("\n");
    printf("-------------------------------------------\n");


    return 0;
}

void testRW(int printResults, int* totalRead, int* totalSuccRead, int* totalWrite, int* totalSuccWrite) {
    unsigned long init_time;
    // Read some short and long address registers and compare it with their default value.
    int contReadReg = 9;
    int contReadSucc = 0;
    if (printResults) {
        // init_time = millis();
        printf("\n");
        printf("Leyendo %i registros...\n", contReadReg);
        printf("\n");
    }
    contReadSucc += assertReadShort("MRF_PACON2", MRF_PACON2, 0b10001000, printResults);
    contReadSucc += assertReadShort("MRF_TXSTBL", MRF_TXSTBL, 0b01110101, printResults);
    contReadSucc += assertReadShort("MRF_TXBCON1", MRF_TXBCON1, 0b00110000, printResults);
    contReadSucc += assertReadLong("MRF_WAKETIMEL", MRF_WAKETIMEL, 0b00001010, printResults);  // Unico reg de direccion larga con valor por defecto distinto de cero
    contReadSucc += assertReadShort("MRF_BBREG2", MRF_BBREG2, 0b01001000, printResults);
    contReadSucc += assertReadShort("MRF_BBREG3", MRF_BBREG3, 0b11011000, printResults);
    contReadSucc += assertReadShort("MRF_BBREG4", MRF_BBREG4, 0b10011100, printResults);
    contReadSucc += assertReadShort("MRF_BBREG6", MRF_BBREG6, 0b00000001, printResults);
    contReadSucc += assertReadShort("MRF_INTCON", MRF_INTCON, 0b11111111, printResults);

    // write and read some short and long address registers
    int contWriteReg = 11;
    int contWriteSucc = 0;
    if (printResults) {
        printf("\n");
        printf("Escribiendo %i registros...\n", contWriteReg);
        printf("\n");
    }

    contWriteSucc += assertWriteShort("MRF_PACON2", MRF_PACON2, 0x98, printResults);
    contWriteSucc += assertWriteShort("MRF_TXSTBL", MRF_TXSTBL, 0x95, printResults);
    contWriteSucc += assertWriteLong("MRF_RFCON0", MRF_RFCON0, 0x03, printResults);
    contWriteSucc += assertWriteLong("MRF_RFCON1", MRF_RFCON1, 0x01, printResults);
    contWriteSucc += assertWriteLong("MRF_RFCON2", MRF_RFCON2, 0x80, printResults);
    contWriteSucc += assertWriteLong("MRF_RFCON6", MRF_RFCON6, 0x90, printResults);
    contWriteSucc += assertWriteLong("MRF_RFCON7", MRF_RFCON7, 0x80, printResults);
    contWriteSucc += assertWriteLong("MRF_RFCON8", MRF_RFCON8, 0x10, printResults);
    contWriteSucc += assertWriteLong("MRF_SLPCON1", MRF_SLPCON1, 0x21, printResults);
    contWriteSucc += assertWriteShort("MRF_BBREG2", MRF_BBREG2, 0x80, printResults);
    contWriteSucc += assertWriteShort("MRF_CCAEDTH", MRF_CCAEDTH, 0x60, printResults);

    if (printResults) {
        printf("\n");
        printf("------------------Resumen------------------\n");
        printf("\n");
        printf("Lecturas exitosas: %i\n", contReadSucc);
        printf("Lecturas fallidas: %i\n", contReadReg - contReadSucc);
        printf("\n");
        printf("Escrituras exitosas:  %i\n", contWriteSucc);
        printf("Escrituras fallidas: %i\n", contWriteReg - contWriteSucc);
        printf("\n");
        // init_time = millis() - init_time;
        // printf("Test duration: " + String(init_time) + " ms");
        // printf("\n");
        printf("-------------------------------------------\n");
    }
    *totalRead = contReadReg;
    *totalSuccRead = contReadSucc;
    *totalWrite = contWriteReg;
    *totalSuccWrite = contWriteSucc;
}

int assertWriteShort(char* name, int address, int toWriteValue, int printResults) {
    mrf.write_short(address, toWriteValue);
    int readValue = mrf.read_short(address);
    if (printResults) {
        printf("%s: write value: %i, read value: %i\n", name, toWriteValue, readValue);
    }
    return toWriteValue == readValue;
}

int assertWriteLong(char* name, int address, int toWriteValue, int printResults) {
    mrf.write_long(address, toWriteValue);
    int readValue = mrf.read_long(address);
    if (printResults) {
        printf("%s: write value: %i, read value: %i\n", name, toWriteValue, readValue);
    }
    return toWriteValue == readValue;
}

int assertReadShort(char* name, int address, int shouldBeValue, int printResults) {
    int readValue = mrf.read_short(address);
    if (printResults) {
        printf("%s: read value: %i, should be: %i\n", name, readValue, shouldBeValue);
    }
    return shouldBeValue == readValue;
}

int assertReadLong(char* name, int address, int shouldBeValue, int printResults) {
    int readValue = mrf.read_long(address);
    if (printResults) {
        printf("%s: read value: %i, should be: %i\n", name, readValue, shouldBeValue);
    }
    return shouldBeValue == readValue;
}

