// #include <Arduino.h>
// #include <mrf24j.h>

// const int pin_reset = 8;
// const int pin_cs = 6;        // default CS pin on ATmega8/168/328
// const int pin_interrupt = 2;  // default interrupt pin on ATmega8/168/328

// // Functions headers
// void showInitReg();
// void printShortReg(String name, int address, int shouldBeValue, int defaultValue);
// void printLongReg(String name, int address, int shouldBeValue, int defaultValue);
// void testRW(int print, int* totalRead, int* totalSuccRead, int* totalWrite, int* totalSuccWrite);
// int assertReadLong(String name, int address, int shouldBeValue, int print);
// int assertReadShort(String name, int address, int shouldBeValue, int print);
// int assertWriteShort(String name, int address, int toWriteValue, int print);
// int assertWriteLong(String name, int address, int toWriteValue, int print);

// Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

// void setup() {
//     Serial.begin(9600);
//     int totalRead = 0, totalReadSucc = 0, totalWrite = 0, totalWriteSucc = 0;
//     int auxTotalRead, auxTotalReadSucc, auxTotalWrite, auxTotalWriteSucc;

//     Serial.println("\nRealizando test de lectura y escritura...");

//     size_t count = 100;
//     unsigned int testDuration = millis();
//     for (size_t i = 0; i < count; i++) {
//         mrf.reset();
//         testRW(0, &auxTotalRead, &auxTotalReadSucc, &auxTotalWrite, &auxTotalWriteSucc);
//         totalRead += auxTotalRead;
//         totalReadSucc += auxTotalRead;
//         totalWrite += auxTotalWrite;
//         totalWriteSucc += auxTotalWriteSucc;
//     }
//     testDuration = millis() - testDuration;

//     Serial.println();
//     Serial.println("------------------Resumen------------------");
//     Serial.println();
//     Serial.println("Lecturas exitosas: " + String(totalReadSucc));
//     Serial.println("Lecturas fallidas: " + String(totalRead - totalReadSucc));
//     Serial.println();
//     Serial.println("Escrituras exitosas: " + String(totalWriteSucc));
//     Serial.println("Escrituras fallidas: " + String(totalWrite - totalWriteSucc));
//     Serial.println();
//     Serial.println("Test duration: " + String(testDuration) + " ms");
//     Serial.println();
//     Serial.println("-------------------------------------------");
// }

// void loop() {
// }

// void testRW(int print, int* totalRead, int* totalSuccRead, int* totalWrite, int* totalSuccWrite) {
//     unsigned long init_time;
//     // Read some short and long address registers and compare it with their default value.
//     int contReadReg = 9;
//     int contReadSucc = 0;
//     if (print) {
//         init_time = millis();
//         Serial.println("\nLeyendo " + String(contReadReg) + " registros...\n");
//     }
//     contReadSucc += assertReadShort("MRF_PACON2", MRF_PACON2, 0b10001000, print);
//     contReadSucc += assertReadShort("MRF_TXSTBL", MRF_TXSTBL, 0b01110101, print);
//     contReadSucc += assertReadShort("MRF_TXBCON1", MRF_TXBCON1, 0b00110000, print);
//     contReadSucc += assertReadLong("MRF_WAKETIMEL", MRF_WAKETIMEL, 0b00001010, print);  // Unico reg de direccion larga con valor por defecto distinto de cero
//     contReadSucc += assertReadShort("MRF_BBREG2", MRF_BBREG2, 0b01001000, print);
//     contReadSucc += assertReadShort("MRF_BBREG3", MRF_BBREG3, 0b11011000, print);
//     contReadSucc += assertReadShort("MRF_BBREG4", MRF_BBREG4, 0b10011100, print);
//     contReadSucc += assertReadShort("MRF_BBREG6", MRF_BBREG6, 0b00000001, print);
//     contReadSucc += assertReadShort("MRF_INTCON", MRF_INTCON, 0b11111111, print);

//     // write and read some short and long address registers
//     int contWriteReg = 11;
//     int contWriteSucc = 0;
//     if (print) {
//         Serial.println();
//         Serial.println("Escribiendo " + String(contWriteReg) + " registros...\n");
//     }

//     contWriteSucc += assertWriteShort("MRF_PACON2", MRF_PACON2, 0x98, print);
//     contWriteSucc += assertWriteShort("MRF_TXSTBL", MRF_TXSTBL, 0x95, print);
//     contWriteSucc += assertWriteLong("MRF_RFCON0", MRF_RFCON0, 0x03, print);
//     contWriteSucc += assertWriteLong("MRF_RFCON1", MRF_RFCON1, 0x01, print);
//     contWriteSucc += assertWriteLong("MRF_RFCON2", MRF_RFCON2, 0x80, print);
//     contWriteSucc += assertWriteLong("MRF_RFCON6", MRF_RFCON6, 0x90, print);
//     contWriteSucc += assertWriteLong("MRF_RFCON7", MRF_RFCON7, 0x80, print);
//     contWriteSucc += assertWriteLong("MRF_RFCON8", MRF_RFCON8, 0x10, print);
//     contWriteSucc += assertWriteLong("MRF_SLPCON1", MRF_SLPCON1, 0x21, print);
//     contWriteSucc += assertWriteShort("MRF_BBREG2", MRF_BBREG2, 0x80, print);
//     contWriteSucc += assertWriteShort("MRF_CCAEDTH", MRF_CCAEDTH, 0x60, print);

//     if (print) {
//         Serial.println();
//         Serial.println("------------------Resumen------------------");
//         Serial.println();
//         Serial.println("Lecturas exitosas: " + String(contReadSucc));
//         Serial.println("Lecturas fallidas: " + String(contReadReg - contReadSucc));
//         Serial.println();
//         Serial.println("Escrituras exitosas: " + String(contWriteSucc));
//         Serial.println("Escrituras fallidas: " + String(contWriteReg - contWriteSucc));
//         Serial.println();
//         init_time = millis() - init_time;
//         Serial.println("Test duration: " + String(init_time) + " ms");
//         Serial.println();
//         Serial.println("-------------------------------------------");
//     }
//     *totalRead = contReadReg;
//     *totalSuccRead = contReadSucc;
//     *totalWrite = contWriteReg;
//     *totalSuccWrite = contWriteSucc;
// }

// int assertWriteShort(String name, int address, int toWriteValue, int print) {
//     mrf.write_short(address, toWriteValue);
//     int readValue = mrf.read_short(address);
//     if (print) {
//         Serial.println(name + ": write value: " + String(toWriteValue) + ", read value: " + String(readValue));
//     }
//     return toWriteValue == readValue;
// }

// int assertWriteLong(String name, int address, int toWriteValue, int print) {
//     mrf.write_long(address, toWriteValue);
//     int readValue = mrf.read_long(address);
//     if (print) {
//         Serial.println(name + ": write value: " + String(toWriteValue) + ", read value: " + String(readValue));
//     }
//     return toWriteValue == readValue;
// }

// int assertReadShort(String name, int address, int shouldBeValue, int print) {
//     int readValue = mrf.read_short(address);
//     if (print) {
//         Serial.println(name + ": read value: " + String(readValue) + ", should be: " + String(shouldBeValue));
//     }
//     return shouldBeValue == readValue;
// }

// int assertReadLong(String name, int address, int shouldBeValue, int print) {
//     int readValue = mrf.read_long(address);
//     if (print) {
//         Serial.println(name + ": read value: " + String(readValue) + ", should be: " + String(shouldBeValue));
//     }
//     return shouldBeValue == readValue;
// }

// void showInitReg() {
//     //    write_short(MRF_PACON2, 0x98); // – Initialize FIFOEN = 1 and TXONTS = 0x6.
//     printShortReg("MRF_PACON2", MRF_PACON2, 0x98, 0b10001000);

//     //    write_short(MRF_TXSTBL, 0x95); // – Initialize RFSTBL = 0x9.
//     printShortReg("MRF_TXSTBL", MRF_TXSTBL, 0x95, 0b01110101);

//     // Long addresses

//     // The unique long address register with non cero default value.
//     printLongReg("TEST", 0x222, 0xAAA, 0b00001010);

//     //    write_long(MRF_RFCON0, 0x03); // – Initialize RFOPT = 0x03.
//     printLongReg("MRF_RFCON0", MRF_RFCON0, 0x03, 0b00000000);

//     //    write_long(MRF_RFCON1, 0x01); // – Initialize VCOOPT = 0x02.
//     printLongReg("MRF_RFCON1", MRF_RFCON1, 0x01, 0b00000000);

//     //    write_long(MRF_RFCON2, 0x80); // – Enable PLL (PLLEN = 1).
//     printLongReg("MRF_RFCON2", MRF_RFCON2, 0x80, 0b00000000);

//     //    write_long(MRF_RFCON6, 0x90); // – Initialize TXFIL = 1 and 20MRECVR = 1.
//     printLongReg("MRF_RFCON6", MRF_RFCON6, 0x90, 0b00000000);

//     //    write_long(MRF_RFCON7, 0x80); // – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
//     printLongReg("MRF_RFCON7", MRF_RFCON7, 0x80, 0b0000000);

//     //    write_long(MRF_RFCON8, 0x10); // – Initialize RFVCO = 1.
//     printLongReg("MRF_RFCON8", MRF_RFCON8, 0x10, 0b00000000);

//     //    write_long(MRF_SLPCON1, 0x21); // – Initialize CLKOUTEN = 1 (disabled) and SLPCLKDIV = 0x01.
//     printLongReg("MRF_SLPCON1", MRF_SLPCON1, 0x21, 0b00000000);

//     //    //  Configuration for nonbeacon-enabled devices (see Section 3.8 “Beacon-Enabled and
//     //    //  Nonbeacon-Enabled Networks”):
//     //    write_short(MRF_BBREG2, 0x80); // Set CCA mode to ED
//     printShortReg("MRF_BBREG2", MRF_BBREG2, 0x80, 0b01001000);

//     //    write_short(MRF_CCAEDTH, 0x60); // – Set CCA ED threshold.
//     printShortReg("MRF_CCAEDTH", MRF_CCAEDTH, 0x60, 0b00000000);

//     //    write_short(MRF_BBREG6, 0x40); // – Set appended RSSI value to RXFIFO.
//     printShortReg("MRF_BBREG6", MRF_BBREG6, 0x41, 0b00000001);
//     //  El primer bit es solo de escritura (es cambiado a 0 automaticamente por hw) y el ultimo de lectura

//     //    set_interrupts();
//     //      interrupts for rx and tx normal complete
//     //      write_short(MRF_INTCON, 0b11110110);
//     printShortReg("MRF_INTCON", MRF_INTCON, 0b11110110, 0b11111111);

//     //    set_channel(12);
//     //      default chanel 11
//     //      write_long(MRF_RFCON0, (((channel - 11) << 4) | 0x03));
//     printLongReg("MRF_RFCON0", MRF_RFCON0, 0b00010011, 0b00000000);

//     //    // max power is by default.. just leave it...
//     //    // Set transmitter power - See “REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)”.

//     //    write_short(MRF_RFCTL, 0x04); //  – Reset RF state machine.
//     //    write_short(MRF_RFCTL, 0x00); // part 2
//     printShortReg("MRF_RFCTL", MRF_RFCTL, 0x00, 0b00000000);
// }

// void printShortReg(String name, int address, int shouldBeValue, int defaultValue) {
//     Serial.print(name + ": ");
//     Serial.print("shouldBeValue: 0x");
//     Serial.print(mrf.read_short(address), HEX);
//     Serial.print(", default: 0x");
//     Serial.print(defaultValue, HEX);
//     Serial.print(", should be: 0x");
//     Serial.println(shouldBeValue, HEX);
// }

// void printLongReg(String name, int address, int shouldBeValue, int defaultValue) {
//     Serial.print(name + ": ");
//     Serial.print("shouldBeValue: 0x");
//     Serial.print(mrf.read_long(address), HEX);
//     Serial.print(", default: 0x");
//     Serial.print(defaultValue, HEX);
//     Serial.print(", should be: 0x");
//     Serial.println(shouldBeValue, HEX);
// }
