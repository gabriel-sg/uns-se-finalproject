#include <mrf24j.h>
#include <Servo.h>

// Functions headers
void interrupt_routine();
void handle_rx();
void handle_tx();
void rw_test(int count, int print_detalles);
void testRW(int print, int* totalRead, int* totalSuccRead, int* totalWrite, int* totalSuccWrite);
int assertReadLong(String name, int address, int shouldBeValue, int print);
int assertReadShort(String name, int address, int shouldBeValue, int print);
int assertWriteShort(String name, int address, int toWriteValue, int print);
int assertWriteLong(String name, int address, int toWriteValue, int print);
void send_pkg(byte sensorId, int value);
void config_mrf(uint16_t panId, uint16_t address, uint8_t channel, boolean promiscuous, boolean palna);
void button();
void read_command();
void do_command(uint8_t actuadorId, uint8_t command);
int adc_to_lux(int adcValue);
int adc_to_temp(int adcValue);

const int pin_reset = 8;
const int pin_cs = 6;
const int pin_interrupt = 2;  // default interrupt pin on ATmega8/168/328

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

Servo groveServo;

unsigned long last_time;
unsigned long tx_interval = 2400;

int luzValue = 0;
// int tempValue = 0;
int sensorPosition = 0;
int shaftPosition = 0;
int lastButtonState = 0;
int toggleValue = 0;

const int pinServo = 3;
int lastServoAngle = 0;
const int potentiometer = A2;

int pin_sensorLuz = A0;
int pin_redLed = 5;
int pin_yellowLed = 4;
int pin_transmisionLed = 7;
int pin_button = 7;
int pin_buzzer = 3;
// int pin_sensorTemp = A3;

// Define the B-value of the thermistor.
// This value is a property of the thermistor used in the Grove - Temperature Sensor,
// and used to convert from the analog value it measures and a temperature value.
const int B = 3975;

// ID actuadores
const uint8_t actuador_redLed = 1;
// const uint8_t actuador_buzzer = 2;
const uint8_t actuador_servo = 2;
const uint8_t actuador_yellowLed = 3;

void setup() {
    Serial.begin(9600);

    int run_test = 0;
    int cant_iteraciones_test = 5;
    int print_detalles = 0;
    if (run_test) {
        rw_test(cant_iteraciones_test, print_detalles);
    }

    uint16_t panId = 0xcafe;
    uint16_t address = 0x6002;
    uint8_t channel = 18;
    boolean promiscuous = false;
    boolean palna = true;
    config_mrf(panId, address, channel, promiscuous, palna);

    Serial.print("PANID attached: ");
    Serial.print(mrf.get_pan(), HEX);
    Serial.println();
    Serial.print("My address: ");
    Serial.print(mrf.address16_read(), HEX);
    Serial.println("\n");

    last_time = millis();

    // Set led (transfer indicator)
    pinMode(pin_transmisionLed, OUTPUT);
    digitalWrite(pin_transmisionLed, LOW);
    // Set led (como actuador)
    pinMode(pin_redLed, OUTPUT);
    digitalWrite(pin_redLed, LOW);
    pinMode(pin_yellowLed, OUTPUT);
    digitalWrite(pin_yellowLed, LOW);
    // Buzzer
    // pinMode(pin_buzzer, OUTPUT);
    // digitalWrite(pin_buzzer, LOW);
    // Set sensor luz
    pinMode(pin_sensorLuz, INPUT);
    // Button
    // pinMode(pin_button, INPUT);
    // Servo
    groveServo.attach(pinServo);
    groveServo.write(0);
    pinMode(potentiometer,INPUT);
}

void interrupt_routine() {
    mrf.interrupt_handler();  // mrf24 object interrupt routine
}

void loop() {
    mrf.check_flags(&handle_rx, &handle_tx);
    unsigned long current_time = millis();
    if ((current_time - last_time) > tx_interval) {
        // Luz
        // noInterrupts();
        luzValue = analogRead(pin_sensorLuz);
        // interrupts();
        luzValue = adc_to_lux(luzValue);
        // Serial.println("Luminocidad: " + String(luzValue) + "\n");
        send_pkg(2, luzValue);

        // delay(15);

        // Tempreratura
        // tempValue = analogRead(pin_sensorTemp);
        // tempValue = adc_to_temp(tempValue);
        //Serial.println("Tempretarura: " + String(tempValue) + "\n");
        // send_pkg(1, tempValue);

        // Servo
        last_time = current_time;
    }
    // sensorPosition = analogRead(potentiometer);
    // shaftPosition = map(sensorPosition, 0, 1023, 0, 179);
    // Use the Servo object to move the servo.
    // groveServo.write(shaftPosition);
    // button();
}

void send_pkg(byte sensorId, int value) {
    digitalWrite(pin_transmisionLed, HIGH);
    // Serial.println("rxxxing...");
    // Serial.println("txxxing...\n");
    mrf.send_value(0x6001, 0, sensorId, 1, value);
    // delay(10);
    digitalWrite(pin_transmisionLed, LOW);
}

void button() {
    int buttonState = digitalRead(pin_button);

    if( buttonState != lastButtonState){
        if (!buttonState) {
            if(toggleValue){
                // digitalWrite(pin_buzzer, 1);
                digitalWrite(pin_redLed, 1);

            }else{
                // digitalWrite(pin_buzzer, 0);
                digitalWrite(pin_redLed, 0);
            }
            toggleValue = !toggleValue;
        }
        lastButtonState = buttonState;
    }
}

int adc_to_lux(int adcValue) {
    int luxValue = 0;
    if (adcValue > 900) {
        luxValue = 100;
    } else if (adcValue > 800) {
        luxValue = map(adcValue, 800, 900, 80, 100);
    } else if (adcValue > 700) {
        luxValue = map(adcValue, 700, 800, 40, 80);
    } else if (adcValue > 600) {
        luxValue = map(adcValue, 600, 700, 20, 40);
    } else if (adcValue > 500) {
        luxValue = map(adcValue, 500, 600, 10, 20);
    } else if (adcValue > 400) {
        luxValue = map(adcValue, 400, 500, 6, 10);
    } else if (adcValue > 300) {
        luxValue = map(adcValue, 300, 400, 3, 6);
    } else if (adcValue > 200) {
        luxValue = map(adcValue, 200, 300, 1, 3);
    } else {
        luxValue = 1;
    }
    return luxValue;
}

int adc_to_temp(int adcValue){
    // Determine the current resistance of the thermistor based on the sensor value.
    float resistance = (float)(1023-adcValue)*10000/adcValue;
    // Calculate the temperature based on the resistance value.
    float temperature = 1/(log(resistance/10000)/B+1/298.15)-273.15;
    return temperature;
}

void config_mrf(uint16_t panId, uint16_t address, uint8_t channel, boolean promiscuous, boolean palna) {
    noInterrupts();
    mrf.reset();
    mrf.init();
    mrf.set_pan(panId);
    // This is _our_ address
    mrf.address16_write(address);
    mrf.set_channel(channel);
    // uncomment if you want to receive any packet on this channel
    mrf.set_promiscuous(promiscuous);
    // uncomment if you want to enable PA/LNA external control
    mrf.set_palna(palna);
    // uncomment if you want to buffer all PHY Payload
    //mrf.set_bufferPHY(true);
    mrf.set_interrupts();
    attachInterrupt(0, interrupt_routine, CHANGE);  // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
    interrupts();
}

void handle_rx() {
    // Serial.print("received a packet ");
    // Serial.print(mrf.get_rxinfo()->frame_length, DEC);
    // Serial.println(" bytes long");

    // if (mrf.get_bufferPHY()) {
    //     Serial.println("Packet data (PHY Payload):");
    //     for (int i = 0; i < mrf.get_rxinfo()->frame_length; i++) {
    //         Serial.print(mrf.get_rxbuf()[i]);
    //     }
    // }

    //#### PRINT PAYLOAD + LQi/rssi ######
    // Serial.println("\r\nBytes data:");
    // for (int i = 0; i < mrf.rx_datalength(); i++) {
    //     Serial.print(String(mrf.get_rxinfo()->rx_data[i]) + "-");
    // }
    // Serial.println("\r\nASCII data:");
    // for (int i = 0; i < mrf.rx_datalength(); i++) {
    //     Serial.write(mrf.get_rxinfo()->rx_data[i]);
    // }
    // Serial.print("\r\nLQI/RSSI=");
    // Serial.print(mrf.get_rxinfo()->lqi, DEC);
    // Serial.print("/");
    // Serial.println(mrf.get_rxinfo()->rssi, DEC);
    // Serial.println();

    read_command();
}

void read_command() {
    uint8_t msg_type = mrf.get_rxinfo()->rx_data[0];
    // Serial.println("MSG TYPE: " + String(msg_type));
    if (msg_type == 1) {  // es un comando
        uint8_t actuadorId = mrf.get_rxinfo()->rx_data[1];
        uint8_t command = mrf.get_rxinfo()->rx_data[2];
        do_command(actuadorId, command);
    }
}

void do_command(uint8_t actuadorId, uint8_t command) {
    // Serial.println("actuadorId: " + String(actuadorId) + " command: " + String(command));
    switch (actuadorId) {
        case actuador_redLed:
            if (command == 1) {
                digitalWrite(pin_redLed, HIGH);
            } else {
                digitalWrite(pin_redLed, LOW);
            }
            break;
        case actuador_servo:
            // Serial.println("Servo command: "+String(command));
            if (command != lastServoAngle){
                // Serial.println("Servo: "+String(command));
                groveServo.write(command);
                lastServoAngle = command;
            }
            break;
        case actuador_yellowLed:
            // Serial.println("Yellow led press: "+String(command));
            digitalWrite(pin_yellowLed, command);
            break;
        default:
            break;
    }
}

void handle_tx() {
    if (mrf.get_txinfo()->tx_ok) {
        // Serial.println("TX went ok, got ack\n");
    } else {
        Serial.print("TX failed after ");
        Serial.print(mrf.get_txinfo()->retries);
        Serial.println(" retries\n");
    }
}

void rw_test(int count, int print_detalles) {
    int totalRead = 0, totalReadSucc = 0, totalWrite = 0, totalWriteSucc = 0;
    int auxTotalRead, auxTotalReadSucc, auxTotalWrite, auxTotalWriteSucc;

    Serial.println("\nRealizando test de lectura y escritura...");

    unsigned int testDuration = millis();
    for (size_t i = 0; i < count; i++) {
        mrf.reset();
        testRW(print_detalles, &auxTotalRead, &auxTotalReadSucc, &auxTotalWrite, &auxTotalWriteSucc);
        totalRead += auxTotalRead;
        totalReadSucc += auxTotalRead;
        totalWrite += auxTotalWrite;
        totalWriteSucc += auxTotalWriteSucc;
    }
    testDuration = millis() - testDuration;

    Serial.println();
    Serial.println("------------------Resumen------------------");
    Serial.println();
    Serial.println("Lecturas exitosas: " + String(totalReadSucc));
    Serial.println("Lecturas fallidas: " + String(totalRead - totalReadSucc));
    Serial.println();
    Serial.println("Escrituras exitosas: " + String(totalWriteSucc));
    Serial.println("Escrituras fallidas: " + String(totalWrite - totalWriteSucc));
    Serial.println();
    Serial.println("Test duration: " + String(testDuration) + " ms");
    Serial.println();
    Serial.println("-------------------------------------------");
}
// Test Functions
void testRW(int print, int* totalRead, int* totalSuccRead, int* totalWrite, int* totalSuccWrite) {
    unsigned long init_time;
    // Read some short and long address registers and compare it with their default value.
    int contReadReg = 9;
    int contReadSucc = 0;
    if (print) {
        init_time = millis();
        Serial.println("\nLeyendo " + String(contReadReg) + " registros...\n");
    }
    contReadSucc += assertReadShort("MRF_PACON2", MRF_PACON2, 0b10001000, print);
    contReadSucc += assertReadShort("MRF_TXSTBL", MRF_TXSTBL, 0b01110101, print);
    contReadSucc += assertReadShort("MRF_TXBCON1", MRF_TXBCON1, 0b00110000, print);
    contReadSucc += assertReadLong("MRF_WAKETIMEL", MRF_WAKETIMEL, 0b00001010, print);  // Unico reg de direccion larga con valor por defecto distinto de cero
    contReadSucc += assertReadShort("MRF_BBREG2", MRF_BBREG2, 0b01001000, print);
    contReadSucc += assertReadShort("MRF_BBREG3", MRF_BBREG3, 0b11011000, print);
    contReadSucc += assertReadShort("MRF_BBREG4", MRF_BBREG4, 0b10011100, print);
    contReadSucc += assertReadShort("MRF_BBREG6", MRF_BBREG6, 0b00000001, print);
    contReadSucc += assertReadShort("MRF_INTCON", MRF_INTCON, 0b11111111, print);

    // write and read some short and long address registers
    int contWriteReg = 11;
    int contWriteSucc = 0;
    if (print) {
        Serial.println();
        Serial.println("Escribiendo " + String(contWriteReg) + " registros...\n");
    }

    contWriteSucc += assertWriteShort("MRF_PACON2", MRF_PACON2, 0x98, print);
    contWriteSucc += assertWriteShort("MRF_TXSTBL", MRF_TXSTBL, 0x95, print);
    contWriteSucc += assertWriteLong("MRF_RFCON0", MRF_RFCON0, 0x03, print);
    contWriteSucc += assertWriteLong("MRF_RFCON1", MRF_RFCON1, 0x01, print);
    contWriteSucc += assertWriteLong("MRF_RFCON2", MRF_RFCON2, 0x80, print);
    contWriteSucc += assertWriteLong("MRF_RFCON6", MRF_RFCON6, 0x90, print);
    contWriteSucc += assertWriteLong("MRF_RFCON7", MRF_RFCON7, 0x80, print);
    contWriteSucc += assertWriteLong("MRF_RFCON8", MRF_RFCON8, 0x10, print);
    contWriteSucc += assertWriteLong("MRF_SLPCON1", MRF_SLPCON1, 0x21, print);
    contWriteSucc += assertWriteShort("MRF_BBREG2", MRF_BBREG2, 0x80, print);
    contWriteSucc += assertWriteShort("MRF_CCAEDTH", MRF_CCAEDTH, 0x60, print);

    if (print) {
        Serial.println();
        Serial.println("------------------Resumen------------------");
        Serial.println();
        Serial.println("Lecturas exitosas: " + String(contReadSucc));
        Serial.println("Lecturas fallidas: " + String(contReadReg - contReadSucc));
        Serial.println();
        Serial.println("Escrituras exitosas: " + String(contWriteSucc));
        Serial.println("Escrituras fallidas: " + String(contWriteReg - contWriteSucc));
        Serial.println();
        init_time = millis() - init_time;
        Serial.println("Test duration: " + String(init_time) + " ms");
        Serial.println();
        Serial.println("-------------------------------------------");
    }
    *totalRead = contReadReg;
    *totalSuccRead = contReadSucc;
    *totalWrite = contWriteReg;
    *totalSuccWrite = contWriteSucc;
}

int assertWriteShort(String name, int address, int toWriteValue, int print) {
    mrf.write_short(address, toWriteValue);
    int readValue = mrf.read_short(address);
    if (print) {
        Serial.println(name + ": write value: " + String(toWriteValue) + ", read value: " + String(readValue));
    }
    return toWriteValue == readValue;
}

int assertWriteLong(String name, int address, int toWriteValue, int print) {
    mrf.write_long(address, toWriteValue);
    int readValue = mrf.read_long(address);
    if (print) {
        Serial.println(name + ": write value: " + String(toWriteValue) + ", read value: " + String(readValue));
    }
    return toWriteValue == readValue;
}

int assertReadShort(String name, int address, int shouldBeValue, int print) {
    int readValue = mrf.read_short(address);
    if (print) {
        Serial.println(name + ": read value: " + String(readValue) + ", should be: " + String(shouldBeValue));
    }
    return shouldBeValue == readValue;
}

int assertReadLong(String name, int address, int shouldBeValue, int print) {
    int readValue = mrf.read_long(address);
    if (print) {
        Serial.println(name + ": read value: " + String(readValue) + ", should be: " + String(shouldBeValue));
    }
    return shouldBeValue == readValue;
}
