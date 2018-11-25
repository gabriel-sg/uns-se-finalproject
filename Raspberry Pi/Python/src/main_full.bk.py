
import pigpio
import time



###### Constantes ######

MRF_RXMCR = 0x00
MRF_PANIDL = 0x01
MRF_PANIDH = 0x02
MRF_SADRL = 0x03
MRF_SADRH = 0x04
MRF_EADR0 = 0x05
MRF_EADR1 = 0x06
MRF_EADR2 = 0x07
MRF_EADR3 = 0x08
MRF_EADR4 = 0x09
MRF_EADR5 = 0x0A
MRF_EADR6 = 0x0B
MRF_EADR7 = 0x0C
MRF_RXFLUSH = 0x0D
# MRF_Reserved = 0x0E
# MRF_Reserved = 0x0F
MRF_ORDER = 0x10
MRF_TXMCR = 0x11
MRF_ACKTMOUT = 0x12
MRF_ESLOTG1 = 0x13
MRF_SYMTICKL = 0x14
MRF_SYMTICKH = 0x15
MRF_PACON0 = 0x16
MRF_PACON1 = 0x17
MRF_PACON2 = 0x18
# MRF_Reserved = 0x19
MRF_TXBCON0 = 0x1A

# TXNCON: TRANSMIT NORMAL FIFO CONTROL REGISTER (ADDRESS: 0x1B)
MRF_TXNCON = 0x1B
MRF_TXNTRIG = 0
MRF_TXNSECEN = 1
MRF_TXNACKREQ = 2
MRF_INDIRECT = 3
MRF_FPSTAT = 4

MRF_TXG1CON = 0x1C
MRF_TXG2CON = 0x1D
MRF_ESLOTG23 = 0x1E
MRF_ESLOTG45 = 0x1F
MRF_ESLOTG67 = 0x20
MRF_TXPEND = 0x21
MRF_WAKECON = 0x22
MRF_FRMOFFSET = 0x23
# TXSTAT: TX MAC STATUS REGISTER (ADDRESS: 0x24)
MRF_TXSTAT = 0x24
TXNRETRY1 = 7
TXNRETRY0 = 6
CCAFAIL = 5
TXG2FNT = 4
TXG1FNT = 3
TXG2STAT = 2
TXG1STAT = 1
TXNSTAT = 0

MRF_TXBCON1 = 0x25
MRF_GATECLK = 0x26
MRF_TXTIME = 0x27
MRF_HSYMTMRL = 0x28
MRF_HSYMTMRH = 0x29
MRF_SOFTRST = 0x2A
# MRF_Reserved = 0x2B
MRF_SECCON0 = 0x2C
MRF_SECCON1 = 0x2D
MRF_TXSTBL = 0x2E
# MRF_Reserved = 0x2F
MRF_RXSR = 0x30
MRF_INTSTAT = 0x31
MRF_INTCON = 0x32
MRF_GPIO = 0x33
MRF_TRISGPIO = 0x34
MRF_SLPACK = 0x35
MRF_RFCTL = 0x36
MRF_SECCR2 = 0x37
MRF_BBREG0 = 0x38
MRF_BBREG1 = 0x39
MRF_BBREG2 = 0x3A
MRF_BBREG3 = 0x3B
MRF_BBREG4 = 0x3C
# MRF_Reserved = 0x3D
MRF_BBREG6 = 0x3E
MRF_CCAEDTH = 0x3F

MRF_RFCON0 = 0x200
MRF_RFCON1 = 0x201
MRF_RFCON2 = 0x202
MRF_RFCON3 = 0x203
MRF_RFCON5 = 0x205
MRF_RFCON6 = 0x206
MRF_RFCON7 = 0x207
MRF_RFCON8 = 0x208
MRF_SLPCAL0 = 0x209
MRF_SLPCAL1 = 0x20A
MRF_SLPCAL2 = 0x20B
MRF_RSSI = 0x210
MRF_SLPCON0 = 0x211
MRF_SLPCON1 = 0x220
MRF_WAKETIMEL = 0x222
MRF_WAKETIMEH = 0x223
MRF_REMCNTL = 0x224
MRF_REMCNTH = 0x225
MRF_MAINCNT0 = 0x226
MRF_MAINCNT1 = 0x227
MRF_MAINCNT2 = 0x228
MRF_MAINCNT3 = 0x229
MRF_TESTMODE = 0x22F
MRF_ASSOEADR1 = 0x231
MRF_ASSOEADR2 = 0x232
MRF_ASSOEADR3 = 0x233
MRF_ASSOEADR4 = 0x234
MRF_ASSOEADR5 = 0x235
MRF_ASSOEADR6 = 0x236
MRF_ASSOEADR7 = 0x237
MRF_ASSOSADR0 = 0x238
MRF_ASSOSADR1 = 0x239
MRF_UPNONCE0 = 0x240
MRF_UPNONCE1 = 0x241
MRF_UPNONCE2 = 0x242
MRF_UPNONCE3 = 0x243
MRF_UPNONCE4 = 0x244
MRF_UPNONCE5 = 0x245
MRF_UPNONCE6 = 0x246
MRF_UPNONCE7 = 0x247
MRF_UPNONCE8 = 0x248
MRF_UPNONCE9 = 0x249
MRF_UPNONCE10 = 0x24A
MRF_UPNONCE11 = 0x24B
MRF_UPNONCE12 = 0x24C

MRF_I_RXIF = 0b00001000
MRF_I_TXNIF = 0b00000001

class rx_info_t:
    def __init__(self):
        self.frame_length = 0
        self.rx_data = []
        for i in range(0,116):
            self.rx_data.append(0)
        self.lqi = 0
        self.rssi = 0

class tx_info_t:
    def __init__(self):
        self.tx_ok = 1
        self.retries = 2
        self.channel_busy = 1

class Mrf24j:

    rx_buf = []
    for i in range(0,127):
        rx_buf.append(0)

    bytes_MHR = 9
    bytes_FCS = 2
    bytes_nodata = bytes_MHR + bytes_FCS

    ignoreBytes = 0

    bufPHY = False

    flag_got_rx = 0
    flag_got_tx = 0

    rx_info = rx_info_t()
    tx_info = tx_info_t()

    pi = pigpio.pi()
    if not pi.connected:
        exit()

    def __init__(self, pin_reset, pin_chip_select, pin_interrupt):
        self._pin_reset = pin_reset
        self._pin_cs = pin_chip_select
        self._pin_int = pin_interrupt
        
        self.pi.set_mode(self._pin_reset,pigpio.OUTPUT)
        self.pi.set_mode(self._pin_cs,pigpio.OUTPUT)
        self.pi.set_mode(self._pin_int,pigpio.INPUT)

        self.spi_handler = self.pi.spi_open(0, 500000, 0) # channel 0 (not use), 500 Kbps, mode 0

    def reset(self):
        self.pi.write(self._pin_reset,pigpio.LOW)
        time.sleep(0.02)
        self.pi.write(self._pin_reset,pigpio.HIGH)
        time.sleep(0.02)

    def spi_transfer(self, byte):
        byte = byte & 255           # truncado a 8 bits
        rx_tuple = self.pi.spi_xfer(self.spi_handler, [byte])
        return rx_tuple[1][0]

    def read_short(self, address):
        self.pi.write(self._pin_cs, pigpio.LOW)
        self.spi_transfer(address << 1 & 0b01111110)
        ret = self.spi_transfer(0)
        self.pi.write(self._pin_cs, pigpio.HIGH)
        return ret

    def read_long(self, address):
        self.pi.write(self._pin_cs, pigpio.LOW)
        ahigh = address >> 3
        alow = address << 5
        self.spi_transfer(0x80 | ahigh)
        self.spi_transfer(alow)
        ret = self.spi_transfer(0)
        self.pi.write(self._pin_cs, pigpio.HIGH)
        return ret

    def write_short(self, address, data):
        self.pi.write(self._pin_cs, pigpio.LOW)
        self.spi_transfer((address<<1 & 0b01111110) | 0x01)
        self.spi_transfer(data)
        self.pi.write(self._pin_cs, pigpio.HIGH)

    def write_long(self, address, data):
        self.pi.write(self._pin_cs, pigpio.LOW)
        ahigh = address >> 3
        alow = address << 5
        self.spi_transfer(0x80 | ahigh)
        self.spi_transfer(alow | 0x10)
        self.spi_transfer(data)
        self.pi.write(self._pin_cs, pigpio.HIGH)

    def init(self):

        self.write_short(MRF_PACON2, 0x98) # – Initialize FIFOEN = 1 and TXONTS = 0x6.
        self.write_short(MRF_TXSTBL, 0x95)# – Initialize RFSTBL = 0x9.

        self.write_long(MRF_RFCON0, 0x03) # – Initialize RFOPT = 0x03.
        self.write_long(MRF_RFCON1, 0x01) # – Initialize VCOOPT = 0x02.
        self.write_long(MRF_RFCON2, 0x80) # – Enable PLL (PLLEN = 1).
        self.write_long(MRF_RFCON6, 0x90) # – Initialize TXFIL = 1 and 20MRECVR = 1.
        self.write_long(MRF_RFCON7, 0x80) # – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
        self.write_long(MRF_RFCON8, 0x10) # – Initialize RFVCO = 1.
        self.write_long(MRF_SLPCON1, 0x21) # – Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.

        self.write_short(MRF_BBREG2, 0x80) # Set CCA mode to ED
        self.write_short(MRF_CCAEDTH, 0x60) # – Set CCA ED threshold.
        self.write_short(MRF_BBREG6, 0x40) # – Set appended RSSI value to RXFIFO.

        self.write_short(MRF_RFCTL, 0x04) #  – Reset RF state machine.
        self.write_short(MRF_RFCTL, 0x00) # part 2
        time.sleep(0.001) # delay at least 192usec

    def get_pan(self):              # return uint16_t
        panh = self.read_short(MRF_PANIDH)
        return panh << 8 | self.read_short(MRF_PANIDL)

    def set_pan(self,panid):         # panid uint16_t
        self.write_short(MRF_PANIDH, panid >> 8)
        self.write_short(MRF_PANIDL, panid & 0xff)

    def address16_write(self,address16):     # addr uint16_t
        self.write_short(MRF_SADRH, address16 >> 8)
        self.write_short(MRF_SADRL, address16 & 0xff)

    def address16_read(self):       # return uint16_t
        a16h = self.read_short(MRF_SADRH)
        return a16h << 8 | self.read_short(MRF_SADRL)

    def set_interrupts(self):
        self.write_short(MRF_INTCON, 0b11110110)

    def set_promiscuous(self,enabled):       # enables bool
        if enabled:
            self.write_short(MRF_RXMCR, 0x01)
        else:
            self.write_short(MRF_RXMCR, 0x00)

    def set_channel(self,channel):           # channel uint8_t
        self.write_long(MRF_RFCON0, (((channel - 11) << 4) | 0x03))

        self.write_short(MRF_RFCTL, 0x04) #  – Reset RF state machine.
        self.write_short(MRF_RFCTL, 0x00) # part 2
        time.sleep(0.001) # delay at least 192usec

    def rx_enable(self):
        self.write_short(MRF_RXFLUSH, 0x01)
        self.write_short(MRF_BBREG1, 0x00)

    def rx_disable(self):
        self.write_short(MRF_BBREG1, 0x04)

    def rx_flush(self):
        self.write_short(MRF_RXFLUSH, 0x01)

    def get_rxinfo(self):           # return rx_info_t *
        return self.rx_info

    def get_txinfo(self):           # return tx_info_t *
        return self.tx_info

    def get_rxbuf(self):            # return uint8_t *
        return self.rx_buf

    def  rx_datalength(self):        # return int
        return (self.rx_info.frame_length - self.bytes_nodata)

    def set_ignoreBytes(self,ib):    # ib int
        self.ignoreBytes = ib

    def set_bufferPHY(self,bp):      # bp bool
        self.bufPHY = bp

    def get_bufferPHY(self):        # return bool
        return self.bufPHY

    def set_palna(self,enabled):     # enabled bool
        if enabled:
            self.write_long(MRF_TESTMODE, 0x07)
        else:
            self.write_long(MRF_TESTMODE, 0x00)

    def send16(self,dest16, data):   # dest16 uint16_t
        data_len = len(data)
        d_send = data.encode()
        i = 0
        self.write_long(i, self.bytes_MHR)
        i += 1
        self.write_long(i, self.bytes_MHR + self.ignoreBytes + data_len)
        i += 1
        self.write_long(i, 0b01100001)
        i += 1
        self.write_long(i, 0b10001000)
        i += 1
        self.write_long(i, 1)
        i += 1

        panid = self.get_pan()

        self.write_long(i, panid & 0xff)
        i += 1
        self.write_long(i, panid >> 8)
        i += 1
        self.write_long(i, dest16 & 0xff)
        i += 1
        self.write_long(i, dest16 >> 8)
        i += 1

        src16 = self.address16_read()
        self.write_long(i, src16 & 0xff)
        i += 1
        self.write_long(i, src16 >> 8)
        i += 1

        i += self.ignoreBytes
        for q in range(0,data_len):
            self.write_long(i, d_send[q])
            i += 1

        self.write_short(MRF_TXNCON, (1<<MRF_TXNACKREQ | 1<<MRF_TXNTRIG))

    def interrupt_handler(self):
        last_interrupt = self.read_short(MRF_INTSTAT)
        # print("last_interrupt: {}".format(hex(last_interrupt)))
        if (last_interrupt & MRF_I_RXIF):
            
            self.rx_disable()

            frame_length = self.read_long(0x300)
            self.rx_info.frame_length = frame_length
            
            if(self.bufPHY):
                rb_ptr = 0
                for i in range(0,frame_length):
                    self.rx_buf[rb_ptr] = self.read_long(0x301 + i)
                    rb_ptr += 1

            rd_ptr = 0
            for i in range(0,self.rx_datalength()):
                self.rx_info.rx_data[rd_ptr] = self.read_long(0x301 + self.bytes_MHR + i)
                rd_ptr += 1

            self.rx_info.lqi = self.read_long(0x301 + frame_length)
            self.rx_info.rssi = self.read_long(0x301 + frame_length + 1)

            self.flag_got_rx += 1
            self.rx_enable()
            # while(self.read_short(MRF_INTSTAT) & 0b00001000):
            #    None

        if (last_interrupt & MRF_I_TXNIF):
            tmp = self.read_short(MRF_TXSTAT)
            # self.tx_info.tx_ok = not(tmp & ~(1 << TXNSTAT))
            self.tx_info.tx_ok = (~tmp & 0x01)
            self.tx_info.retries = tmp >> 6
            self.tx_info.channel_busy = (tmp & (1 << CCAFAIL))
            self.flag_got_tx += 1

    def check_flags(self,rx_handler, tx_handler):
        if (self.flag_got_rx):
            self.flag_got_rx = 0
            rx_handler()
        elif (self.flag_got_tx):
            self.flag_got_tx = 0
            tx_handler()

    def close(self):
        self.pi.spi_close(self.spi_handler)
        self.pi.stop()







###### Raspberry Pi GPIO ######
pin_reset = 16
pin_cs = 12
pin_interrupt = 20
spi_channel = 0

# Connect to PIGPIO Daemon
pi = pigpio.pi()
if not pi.connected:
    exit()

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
    
    contReadSucc += assertReadShort("MRF_PACON2", MRF_PACON2, 0b10001000, printResults)
    contReadSucc += assertReadShort("MRF_TXSTBL", MRF_TXSTBL, 0b01110101, printResults)
    contReadSucc += assertReadShort("MRF_TXBCON1", MRF_TXBCON1, 0b00110000, printResults)
    contReadSucc += assertReadLong("MRF_WAKETIMEL", MRF_WAKETIMEL, 0b00001010, printResults)
    contReadSucc += assertReadShort("MRF_BBREG2", MRF_BBREG2, 0b01001000, printResults)
    contReadSucc += assertReadShort("MRF_BBREG3", MRF_BBREG3, 0b11011000, printResults)
    contReadSucc += assertReadShort("MRF_BBREG4", MRF_BBREG4, 0b10011100, printResults)
    contReadSucc += assertReadShort("MRF_BBREG6", MRF_BBREG6, 0b00000001, printResults)
    contReadSucc += assertReadShort("MRF_INTCON", MRF_INTCON, 0b11111111, printResults)

    contWriteReg = 11
    contWriteSucc = 0
    if (printResults):
        print("\nEscribiendo " + str(contWriteReg) + " registros...\n")

    contWriteSucc += assertWriteShort("MRF_PACON2", MRF_PACON2, 0x98, printResults)
    contWriteSucc += assertWriteShort("MRF_TXSTBL", MRF_TXSTBL, 0x95, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON0", MRF_RFCON0, 0x03, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON1", MRF_RFCON1, 0x01, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON2", MRF_RFCON2, 0x80, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON6", MRF_RFCON6, 0x90, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON7", MRF_RFCON7, 0x80, printResults)
    contWriteSucc += assertWriteLong("MRF_RFCON8", MRF_RFCON8, 0x10, printResults)
    contWriteSucc += assertWriteLong("MRF_SLPCON1", MRF_SLPCON1, 0x21, printResults)
    contWriteSucc += assertWriteShort("MRF_BBREG2", MRF_BBREG2, 0x80, printResults)
    contWriteSucc += assertWriteShort("MRF_CCAEDTH", MRF_CCAEDTH, 0x60, printResults)

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

###### HANDLERS ######

def handle_rx():
    print("received a packet {} bytes long\n".format(mrf.get_rxinfo().frame_length))
    
    if (mrf.get_bufferPHY()):
        print("Packet data (PHY Payload):\n")
        for i in range(0,mrf.get_rxinfo().frame_length):
            print(mrf.get_rxbuf()[i])
    
    print("\r\nASCII data (relevant data):\n")
    for i in range(0,mrf.rx_datalength()):
        print(str(mrf.get_rxinfo().rx_data[i]),end="")
    
    print("\r\nLQI/RSSI=")
    print(mrf.get_rxinfo().lqi)
    print("/")
    print(mrf.get_rxinfo().rssi)

def handle_tx():
    print("handle_tx()" + str(mrf.get_txinfo().tx_ok))
    if (mrf.get_txinfo().tx_ok):
        print("TX went ok, got ack")
    else:
        print("TX failed after {} retries\n".format(mrf.get_txinfo().retries))

def interrupt_routine(gpio,level,tick):
    mrf.interrupt_handler()
    print("Ocurrio una interrupcion")

def dummy_cb(gpio,level,tick):
    print("Ocurrio una dummy")

###### MAIN ######

last_time = 0
tx_interval = 2000

mrf = Mrf24j(pin_reset, pin_cs, pin_interrupt)

if (run_test):
    mrf.reset()
    mrf.init()

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

###### Módulo MRF24J40 ######

mrf.reset()
mrf.init()
mrf.set_channel(18)
mrf.set_pan(0xcafe)
mrf.address16_write(0x6001)
mrf.set_promiscuous(False)
mrf.set_palna(True)
mrf.set_interrupts()
# mrf.set_bufferPHY(true)
# Set interrupt edge rising. (default = falling)
# mrf.write_long(MRF_SLPCON0, 0b00000010)

###### Attach Interruption Handler ######
cb0 = pi.callback(pin_interrupt,pigpio.EITHER_EDGE,interrupt_routine)
cb1 = pi.callback(4,pigpio.EITHER_EDGE,dummy_cb)

print("PANId: 0x{:04X} - Addr: 0x{:04X}".format(mrf.get_pan(),mrf.address16_read()))

last_time = int(round(time.time() * 1000))

# LOOP

while True:
    try:
        mrf.check_flags(handle_rx, handle_tx)
        current_time = int(round(time.time() * 1000))
        if (current_time - last_time > tx_interval):
            last_time = current_time
            print("rxxxing...\n")
    #     mrf.send16(0x6005, "puto el que lee porque el que lee se la come")
    # time.sleep(1)
    
        # time.sleep(60)
    except KeyboardInterrupt:
        break

print("\nSaliendo...")
   
mrf.close()

cb0.cancel()
cb1.cancel()

pi.stop()

# END MAIN






