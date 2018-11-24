#!/usr/bin/env python

import mfr24j40
import pigpio
import time

pin_reset = 16
pin_cs = 12
pin_interrupt = 20
spi_channel = 0

run_test = 1

totalRead = 0
totalReadSucc = 0
totalWrite = 0
totalWriteSucc = 0

# Test Functions
def testRW(printResults):
    init_time = 0
    contReadReg = 9
    contReadSucc = 0
    if (printResults):
        init_time = int(round(time.time() * 1000))
        print("\nLeyendo " + str(contReadReg) + " registros...\n")
    
    contReadSucc += assertReadShort("MRF_PACON2", mfr24j40.MRF_PACON2, 0b10001000, printResults)
    contReadSucc += assertReadShort("MRF_TXSTBL", mfr24j40.MRF_TXSTBL, 0b01110101, printResults)
    contReadSucc += assertReadShort("MRF_TXBCON1", mfr24j40.MRF_TXBCON1, 0b00110000, printResults)
    contReadSucc += assertReadLong("MRF_WAKETIMEL", mfr24j40.MRF_WAKETIMEL, 0b00001010, printResults)
    contReadSucc += assertReadShort("MRF_BBREG2", mfr24j40.MRF_BBREG2, 0b01001000, printResults)
    contReadSucc += assertReadShort("MRF_BBREG3", mfr24j40.MRF_BBREG3, 0b11011000, printResults)
    contReadSucc += assertReadShort("MRF_BBREG4", mfr24j40.MRF_BBREG4, 0b10011100, printResults)
    contReadSucc += assertReadShort("MRF_BBREG6", mfr24j40.MRF_BBREG6, 0b00000001, printResults)
    contReadSucc += assertReadShort("MRF_INTCON", mfr24j40.MRF_INTCON, 0b11111111, printResults)

    contWriteReg = 11
    contWriteSucc = 0
    if (printResults):
        print("\nEscribiendo " + str(contWriteReg) + " registros...\n")

    contWriteSucc += assertWriteShort("MRF_PACON2", mfr24j40.MRF_PACON2, 0x98, printResults)
    contWriteSucc += assertWriteShort("MRF_TXSTBL", mfr24j40.MRF_TXSTBL, 0x95, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON0", mfr24j40.MRF_RFCON0, 0x03, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON1", mfr24j40.MRF_RFCON1, 0x01, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON2", mfr24j40.MRF_RFCON2, 0x80, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON6", mfr24j40.MRF_RFCON6, 0x90, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON7", mfr24j40.MRF_RFCON7, 0x80, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON8", mfr24j40.MRF_RFCON8, 0x10, printResults)
    contWriteSucc += assertWriteLong("MRF_SLPCON1", mfr24j40.MRF_SLPCON1, 0x21, printResults)
    contWriteSucc += assertWriteShort("MRF_BBREG2", mfr24j40.MRF_BBREG2, 0x80, printResults)
    contWriteSucc += assertWriteShort("MRF_CCAEDTH", mfr24j40.MRF_CCAEDTH, 0x60, printResults)

    if (printResults):
        print("\n------------------Resumen------------------\n")
        print("Lecturas exitosas: " + str(contReadSucc))
        print("Lecturas fallidas: " + str(contReadReg - contReadSucc))
        print("\n")
        print("Escrituras exitosas: " + str(contWriteSucc))
        print("Escrituras fallidas: " + str(contWriteReg - contWriteSucc))
        print("\n")
        init_time = int(round(time.time() * 1000)) - init_time
        print("Test duration: " + str(init_time) + " ms\n")
        print("-------------------------------------------")

    #totalRead += contReadReg
    #totalReadSucc += contReadSucc
    #totalWrite += contWriteReg
    #totalWriteSucc += contWriteSucc

def assertWriteShort(name, address, toWriteValue, printResults):
    mrf.write_short(address, toWriteValue)
    readValue = mrf.read_short(address)
    if (printResults):
        print(str(name) + ": write value: " + str(toWriteValue) + ", read value: " + str(readValue))
    return toWriteValue == readValue

def assertWriteLong(name, address, toWriteValue, printResults):
    mrf.write_long(address, toWriteValue)
    readValue = mrf.read_long(address)
    if (printResults):
        print(str(name) + ": write value: " + str(toWriteValue) + ", read value: " + str(readValue))
    return toWriteValue == readValue

def assertReadShort(name, address, shouldBeValue, printResults):
    readValue = mrf.read_short(address)
    if (printResults):
        print(str(name) + ": read value: " + str(readValue) + ", should be: " + str(shouldBeValue))
    return shouldBeValue == readValue

def assertReadLong(name, address, shouldBeValue, printResults):
    readValue = mrf.read_long(address)
    if (printResults):
        print(str(name) + ": read value: " + str(readValue) + ", should be: " + str(shouldBeValue))
    return shouldBeValue == readValue

# HANDLERS

def handle_rx():
    print("received a packet {} bytes long\n".format(mrf.get_rxinfo().frame_length))
    
    if (mrf.get_bufferPHY()):
        print("Packet data (PHY Payload):\n")
        for i in range(0,mrf.get_rxinfo().frame_length):
            print(mrf.get_rxbuf()[i])
    
    print("\r\nASCII data (relevant data):\n")
    for i in range(0,mrf.rx_datalength()):
        print(mrf.get_rxinfo().rx_data[i])
    
    print("\r\nLQI/RSSI=")
    print(mrf.get_rxinfo().lqi)
    print("/")
    print(mrf.get_rxinfo().rssi)

def handle_tx():
    if (mrf.get_txinfo().tx_ok):
        print("TX went ok, got ack\n")
    else:
        print("TX failed after {} retries\n\n".format(mrf.get_txinfo().retries))

def interrupt_routine(gpio,level,tick):
    mrf.interrupt_handler()
    print("Ocurrio una interrupcion\n")

pi = pigpio.pi()
if not pi.connected:
    exit()

mrf = mfr24j40.Mrf24j(pin_reset, pin_cs, pin_interrupt)

last_time = 0
tx_interval = 2000

mrf.reset()
mrf.init()

mrf.set_pan(0xcafe)
mrf.address16_write(0x6001)

#mrf.set_promiscuous(false)
mrf.set_palna(True)

#mrf.set_channel(18)
#mrf.set_bufferPHY(true)
# Set interrupt edge rising. (default = falling)
# mrf.write_long(MRF_SLPCON0, 0b00000010)

pi.callback(pin_interrupt,pigpio.FALLING_EDGE,interrupt_routine)

print("PANId: 0x{:04X} - Addr: 0x{:04X}".format(mrf.get_pan(),mrf.address16_read()))

last_time = int(round(time.time() * 1000))

if (run_test):

    print("\nRealizando test de lectura y escritura...")

    count = 10
    testDuration = int(round(time.time() * 1000))
    for i in range(0,count):
        mrf.reset()
        testRW(1)

    testDuration = int(round(time.time() * 1000)) - testDuration

    print("\n------------------Resumen------------------\n")
    print("Lecturas exitosas: " + str(totalReadSucc))
    print("Lecturas fallidas: " + str(totalRead - totalReadSucc))
    print("\n")
    print("Escrituras exitosas: " + str(totalWriteSucc))
    print("Escrituras fallidas: " + str(totalWrite - totalWriteSucc))
    print("\n")
    print("Test duration: " + str(testDuration) + " ms")
    print("\n")
    print("-------------------------------------------")

    mrf.reset()
    mrf.init()

# LOOP

while True:
    mrf.check_flags(handle_rx, handle_tx)
    current_time = int(round(time.time() * 1000))
    if (current_time - last_time > tx_interval):
        last_time = current_time
        print("txxxing...\n")
        mrf.send16(0x6005, "abcd")
    time.sleep(1)
    
# END MAIN