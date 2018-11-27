import wiringpi
import time
import mrf24j40_wiringPi

PANID = 0xcafe
ADDR_NODO_1 = 0x6001
ADDR_NODO_2 = 0x6002
ADDR_NODO_3 = 0x6003

# init wiringpi
wiringpi.wiringPiSetup() # wiringpi pin's number
# wiringpi.wiringPiSetupGpio() # BCM pin's number

###### Raspberry Pi GPIO ######
pin_reset = 27
pin_cs = 26
pin_interrupt = 28

mrf = mrf24j40_wiringPi.Mrf24j(pin_reset, pin_cs, pin_interrupt)

###### Test Functions ######
run_test = 1

totalRead = 0
totalReadSucc = 0
totalWrite = 0
totalWriteSucc = 0

def testRW(printResults):
    init_time = 0
    contReadReg = 9
    contReadSucc = 0
    if (printResults):
        init_time = int(round(time.time() * 1000))
        print("\nLeyendo " + str(contReadReg) + " registros...\n")

    contReadSucc += assertReadShort("MRF_PACON2", mrf24j40_wiringPi.MRF_PACON2, 0b10001000, printResults)
    contReadSucc += assertReadShort("MRF_TXSTBL", mrf24j40_wiringPi.MRF_TXSTBL, 0b01110101, printResults)
    contReadSucc += assertReadShort("MRF_TXBCON1", mrf24j40_wiringPi.MRF_TXBCON1, 0b00110000, printResults)
    contReadSucc += assertReadLong("MRF_WAKETIMEL", mrf24j40_wiringPi.MRF_WAKETIMEL, 0b00001010, printResults)
    contReadSucc += assertReadShort("MRF_BBREG2", mrf24j40_wiringPi.MRF_BBREG2, 0b01001000, printResults)
    contReadSucc += assertReadShort("MRF_BBREG3", mrf24j40_wiringPi.MRF_BBREG3, 0b11011000, printResults)
    contReadSucc += assertReadShort("MRF_BBREG4", mrf24j40_wiringPi.MRF_BBREG4, 0b10011100, printResults)
    contReadSucc += assertReadShort("MRF_BBREG6", mrf24j40_wiringPi.MRF_BBREG6, 0b00000001, printResults)
    contReadSucc += assertReadShort("MRF_INTCON", mrf24j40_wiringPi.MRF_INTCON, 0b11111111, printResults)

    contWriteReg = 11
    contWriteSucc = 0
    if (printResults):
        print("\nEscribiendo " + str(contWriteReg) + " registros...\n")

    contWriteSucc += assertWriteShort("MRF_PACON2", mrf24j40_wiringPi.MRF_PACON2, 0x98, printResults)
    contWriteSucc += assertWriteShort("MRF_TXSTBL", mrf24j40_wiringPi.MRF_TXSTBL, 0x95, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON0", mrf24j40_wiringPi.MRF_RFCON0, 0x03, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON1", mrf24j40_wiringPi.MRF_RFCON1, 0x01, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON2", mrf24j40_wiringPi.MRF_RFCON2, 0x80, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON6", mrf24j40_wiringPi.MRF_RFCON6, 0x90, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON7", mrf24j40_wiringPi.MRF_RFCON7, 0x80, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON8", mrf24j40_wiringPi.MRF_RFCON8, 0x10, printResults)
    contWriteSucc += assertWriteLong("MRF_SLPCON1", mrf24j40_wiringPi.MRF_SLPCON1, 0x21, printResults)
    contWriteSucc += assertWriteShort("MRF_BBREG2", mrf24j40_wiringPi.MRF_BBREG2, 0x80, printResults)
    contWriteSucc += assertWriteShort("MRF_CCAEDTH", mrf24j40_wiringPi.MRF_CCAEDTH, 0x60, printResults)

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

    global totalRead, totalReadSucc, totalWrite, totalWriteSucc
    totalRead += contReadReg
    totalReadSucc += contReadSucc
    totalWrite += contWriteReg
    totalWriteSucc += contWriteSucc

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

###### HANDLERS ######

def interrupt_routine():
    print("\nOcurrio una interrupcion")
    mrf.interrupt_handler()

def handle_rx():
    print("received a packet {} bytes long\n".format(mrf.get_rxinfo().frame_length))

    if (mrf.get_bufferPHY()):
        print("Packet data (PHY Payload):\n")
        for i in range(0,mrf.get_rxinfo().frame_length):
            print(mrf.get_rxbuf()[i])

    print("\r\nASCII data (relevant data):\n")
    for i in range(0,mrf.rx_datalength()):
        print(str(mrf.get_rxinfo().rx_data[i])+"-",end="")

    print("\r\nLQI/RSSI=", end="")
    print(mrf.get_rxinfo().lqi, end="")
    print("/", end="")
    print(mrf.get_rxinfo().rssi)

def handle_tx():
    print("handle_tx()" + str(mrf.get_txinfo().tx_ok))
    if (mrf.get_txinfo().tx_ok):
        print("TX went ok, got ack")
    else:
        print("TX failed after {} retries\n".format(mrf.get_txinfo().retries))


###### MAIN ######

last_time = 0
tx_interval = 2000

if (run_test):
    print("\nRealizando test de lectura y escritura...")
    count = 1
    testDuration = int(round(time.time() * 1000))
    for i in range(0,count):
        mrf.reset()
        testRW(1)

    testDuration = int(round(time.time() * 1000)) - testDuration

    print("\n---------------Resumen Total---------------\n")
    print("Lecturas exitosas: " + str(totalReadSucc))
    print("Lecturas fallidas: " + str(totalRead - totalReadSucc))
    print("\n")
    print("Escrituras exitosas: " + str(totalWriteSucc))
    print("Escrituras fallidas: " + str(totalWrite - totalWriteSucc))
    print("\n")
    print("Test duration: " + str(testDuration) + " ms")
    print("-------------------------------------------")


###### Módulo MRF24J40 ######
mrf.reset()
mrf.init()

mrf.set_channel(18)
mrf.set_pan(PANID)
mrf.address16_write(ADDR_NODO_1)
mrf.set_promiscuous(False)
mrf.set_palna(True)
mrf.set_interrupts()
time.sleep(0.02)
assertReadShort("MRF_INTCON", mrf24j40_wiringPi.MRF_INTCON, 0b11110110, 1)

###### Attach Interruption Handler ######
wiringpi.wiringPiISR(pin_interrupt, wiringpi.GPIO.INT_EDGE_FALLING, interrupt_routine)

print("PANId: 0x{:04X} - Addr: 0x{:04X}".format(mrf.get_pan(),mrf.address16_read()))

last_time = int(round(time.time() * 1000))

###### LOOP ######

while True:
    try:
        mrf.check_flags(handle_rx, handle_tx)
        current_time = int(round(time.time() * 1000))
        if (current_time - last_time > tx_interval):
            last_time = current_time
            print("txxxing...\n")
            mrf.send16(0x6003, "puto el que lee porque el que lee se la come")
            # time.sleep(1)

        # time.sleep(60)
    except KeyboardInterrupt:
        break

print("\nSaliendo...")

print("fin.")

###### END MAIN ######
