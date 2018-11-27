// /**
//  * Example code for using a microchip mrf24j40 module to send and receive
//  * packets using plain 802.15.4
//  * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
//  * notifications
//  * This example file is considered to be in the public domain
//  * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
//  */
// #include <SPI.h>
// #include <mrf24j.h>

// // Functions headers
// void interrupt_routine();
// void handle_rx();
// void handle_tx();

// void rw_test(int count, int print_detalles);
// void testRW(int print, int* totalRead, int* totalSuccRead, int* totalWrite, int* totalSuccWrite);
// int assertReadLong(String name, int address, int shouldBeValue, int print);
// int assertReadShort(String name, int address, int shouldBeValue, int print);
// int assertWriteShort(String name, int address, int toWriteValue, int print);
// int assertWriteLong(String name, int address, int toWriteValue, int print);
// void send_pkg(byte sensorId, int value);
// void config_mrf(uint16_t panId, uint16_t address, uint8_t channel, boolean promiscuous, boolean palna);
// void button();

// const int pin_reset = 8;
// const int pin_cs = 6;         // default CS pin on ATmega8/168/328
// const int pin_interrupt = 2;  // default interrupt pin on ATmega8/168/328

// Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

// unsigned long last_time;
// unsigned long tx_interval = 5000;

// int luzValue = 0;

// int pin_blueLed = 5;
// int pin_greenLed = 4;
// int pin_sensorLuz = A0;
// int pin_button = 7;
// int pin_buzzer = 3;

// void setup() {
//     Serial.begin(9600);

//     int run_test = 1;
//     int cant_iteraciones_test = 5;
//     int print_detalles = 0;
//     if(run_test){
//         rw_test(cant_iteraciones_test, print_detalles);
//     }

//     uint16_t panId = 0xcafe;
//     uint16_t address = 0x6003;
//     uint8_t channel = 18;
//     boolean promiscuous = false;
//     boolean palna = true;
//     config_mrf(panId, address, channel, promiscuous, palna);
    
//     Serial.print("PANID attached: ");
//     Serial.print(mrf.get_pan(), HEX);
//     Serial.println();
//     Serial.print("My address: ");
//     Serial.print(mrf.address16_read(), HEX);
//     Serial.println("\n");

//     last_time = millis();

//     // Set led (transfer indicator)
//     pinMode(pin_greenLed, OUTPUT);
//     digitalWrite(pin_greenLed, LOW);
//     // Set led (como actuador)
//     pinMode(pin_blueLed, OUTPUT);
//     digitalWrite(pin_blueLed, LOW);
//     // Buzzer
//     pinMode(pin_buzzer, OUTPUT);
//     digitalWrite(pin_buzzer, LOW);
//     // Set sensor luz
//     pinMode(pin_sensorLuz, INPUT);
//     // Button
//     pinMode(pin_button, INPUT);
// }

// void interrupt_routine() {
//     mrf.interrupt_handler();  // mrf24 object interrupt routine
// }

// void loop() {
//     mrf.check_flags(&handle_rx, &handle_tx);
//     unsigned long current_time = millis();
//     if ((current_time - last_time) > tx_interval) {
//         // Luz
//         noInterrupts();
//         luzValue = analogRead(A0);
//         interrupts();
//         Serial.println("Luminocidad: "+ String(luzValue)+"\n");
//         send_pkg(2, luzValue);        
//         last_time = current_time;
//     }
//     button();
// }

// void send_pkg(byte sensorId, int value){
//     digitalWrite(pin_greenLed, HIGH);
//     //Serial.println("rxxxing...");
//     Serial.println("txxxing...\n");
//     mrf.send_value(0x6001, 0, sensorId, 1, value);
//     delay(10);
//     digitalWrite(pin_greenLed, LOW);
// }

// void button(){
//     int buttonState = digitalRead(pin_button);
//     if(buttonState == 1){
//         digitalWrite(pin_buzzer, HIGH);
//         digitalWrite(pin_blueLed, HIGH);
//     }
//     else{
//         digitalWrite(pin_buzzer, LOW);
//         digitalWrite(pin_blueLed, LOW);
//     }
// }

// void config_mrf(uint16_t panId, uint16_t address, uint8_t channel, boolean promiscuous, boolean palna){
//     noInterrupts();
//     mrf.reset();
//     mrf.init();
//     mrf.set_pan(panId);
//     // This is _our_ address
//     mrf.address16_write(address);
//     mrf.set_channel(channel);
//     // uncomment if you want to receive any packet on this channel
//     mrf.set_promiscuous(promiscuous);
//     // uncomment if you want to enable PA/LNA external control
//     mrf.set_palna(palna);
//     // uncomment if you want to buffer all PHY Payload
//     //mrf.set_bufferPHY(true);
//     mrf.set_interrupts();
//     attachInterrupt(0, interrupt_routine, CHANGE);  // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
//     interrupts();
// }

// void handle_rx() {
//     Serial.print("received a packet ");
//     Serial.print(mrf.get_rxinfo()->frame_length, DEC);
//     Serial.println(" bytes long");

//     if (mrf.get_bufferPHY()) {
//         Serial.println("Packet data (PHY Payload):");
//         for (int i = 0; i < mrf.get_rxinfo()->frame_length; i++) {
//             Serial.print(mrf.get_rxbuf()[i]);
//         }
//     }

//     Serial.println("ASCII data (relevant data):");
//     for (int i = 0; i < mrf.rx_datalength(); i++) {
//         Serial.write(mrf.get_rxinfo()->rx_data[i]);
//     }
//     Serial.println("\r\nBytes data:");
//     for (int i = 0; i < mrf.rx_datalength(); i++) {
//         Serial.print(String(mrf.get_rxinfo()->rx_data[i])+"-");
//     }

//     Serial.print("\r\nLQI/RSSI=");
//     Serial.print(mrf.get_rxinfo()->lqi, DEC);
//     Serial.print("/");
//     Serial.println(mrf.get_rxinfo()->rssi, DEC);
//     Serial.println();
// }

// void handle_tx() {
//     if (mrf.get_txinfo()->tx_ok) {
//         Serial.println("TX went ok, got ack\n");
//     } else {
//         Serial.print("TX failed after ");
//         Serial.print(mrf.get_txinfo()->retries);
//         Serial.println(" retries\n");
//     }
// }

// void rw_test(int count, int print_detalles) {
//     int totalRead = 0, totalReadSucc = 0, totalWrite = 0, totalWriteSucc = 0;
//     int auxTotalRead, auxTotalReadSucc, auxTotalWrite, auxTotalWriteSucc;

//     Serial.println("\nRealizando test de lectura y escritura...");

//     unsigned int testDuration = millis();
//     for (size_t i = 0; i < count; i++) {
//         mrf.reset();
//         testRW(print_detalles, &auxTotalRead, &auxTotalReadSucc, &auxTotalWrite, &auxTotalWriteSucc);
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
// // Test Functions
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
