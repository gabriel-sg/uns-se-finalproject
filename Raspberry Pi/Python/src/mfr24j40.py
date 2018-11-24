import pigpio

#Constantes

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
#MRF_Reserved = 0x0E
#MRF_Reserved = 0x0F
MRF_ORDER = 0x10
MRF_TXMCR = 0x11
MRF_ACKTMOUT = 0x12
MRF_ESLOTG1 = 0x13
MRF_SYMTICKL = 0x14
MRF_SYMTICKH = 0x15
MRF_PACON0 = 0x16
MRF_PACON1 = 0x17
MRF_PACON2 = 0x18
#MRF_Reserved = 0x19
MRF_TXBCON0 = 0x1A

#TXNCON: TRANSMIT NORMAL FIFO CONTROL REGISTER (ADDRESS: 0x1B)
MRF_TXNCON = 0x1B
MRF_TXNTRIG   = 0
MRF_TXNSECEN  = 1
MRF_TXNACKREQ = 2
MRF_INDIRECT  = 3
MRF_FPSTAT    = 4

MRF_TXG1CON = 0x1C
MRF_TXG2CON = 0x1D
MRF_ESLOTG23 = 0x1E
MRF_ESLOTG45 = 0x1F
MRF_ESLOTG67 = 0x20
MRF_TXPEND = 0x21
MRF_WAKECON = 0x22
MRF_FRMOFFSET = 0x23
#TXSTAT: TX MAC STATUS REGISTER (ADDRESS: 0x24)
MRF_TXSTAT = 0x24
TXNRETRY1     = 7
TXNRETRY0     = 6
CCAFAIL       = 5
TXG2FNT       = 4
TXG1FNT       = 3
TXG2STAT      = 2
TXG1STAT      = 1
TXNSTAT       = 0

MRF_TXBCON1 = 0x25
MRF_GATECLK = 0x26
MRF_TXTIME = 0x27
MRF_HSYMTMRL = 0x28
MRF_HSYMTMRH = 0x29
MRF_SOFTRST = 0x2A
#MRF_Reserved = 0x2B
MRF_SECCON0 = 0x2C
MRF_SECCON1 = 0x2D
MRF_TXSTBL = 0x2E
#MRF_Reserved = 0x2F
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
#MRF_Reserved = 0x3D
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

# typedef struct _rx_info_t{
#     uint8_t frame_length;
#     uint8_t rx_data[116]; //max data length = (127 aMaxPHYPacketSize - 2 Frame control - 1 sequence number - 2 panid - 2 shortAddr Destination - 2 shortAddr Source - 2 FCS)
#     uint8_t lqi;
#     uint8_t rssi;
# } rx_info_t;

class rx_info_t:
    def __init__(self):
        self.frame_length = 0
        self.rx_data = []
        self.lqi = 0
        self.rssi = 0

class tx_info_t:
    def __init__(self):
        self.tx_ok = 1
        self.retries = 2
        self.channel_busy = 1

class Mrf24j:

    pi = pigpio.pi()

    if not pi.connected:
        exit()
    
    def __init__(self,pin_reset, pin_chip_select, pin_interrupt):
        
        self._pin_reset = pin_reset
        self._pin_cs = pin_chip_select
        self._pin_int = pin_interrupt

        self.pi.set_mode(self._pin_reset,pigpio.OUTPUT)
        self.pi.set_mode(self._pin_cs,pigpio.OUTPUT)
        self.pi.set_mode(self._pin_int,pigpio.INPUT)

    def reset():
    def init():
        
        write_short(MRF_PACON2, 0x98) # – Initialize FIFOEN = 1 and TXONTS = 0x6.
        write_short(MRF_TXSTBL, 0x95)# – Initialize RFSTBL = 0x9.

        write_long(MRF_RFCON0, 0x03) # – Initialize RFOPT = 0x03.
        write_long(MRF_RFCON1, 0x01) # – Initialize VCOOPT = 0x02.
        write_long(MRF_RFCON2, 0x80) # – Enable PLL (PLLEN = 1).
        write_long(MRF_RFCON6, 0x90) # – Initialize TXFIL = 1 and 20MRECVR = 1.
        write_long(MRF_RFCON7, 0x80) # – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
        write_long(MRF_RFCON8, 0x10) # – Initialize RFVCO = 1.
        write_long(MRF_SLPCON1, 0x21) # – Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.

        write_short(MRF_BBREG2, 0x80) # Set CCA mode to ED
        write_short(MRF_CCAEDTH, 0x60) # – Set CCA ED threshold.
        write_short(MRF_BBREG6, 0x40) # – Set appended RSSI value to RXFIFO.
        
        set_channel(12)

        write_short(MRF_RFCTL, 0x04) #  – Reset RF state machine.
        write_short(MRF_RFCTL, 0x00) # part 2
        sleep(0.001) # delay at least 192usec

    byte read_short(byte address);
    byte read_long(word address);

    void write_short(byte address, byte data);
    void write_long(word address, byte data);

    word get_pan(void);
    void set_pan(word panid);

    void address16_write(word address16);
    word address16_read(void);

    void set_interrupts(void);

    void set_promiscuous(boolean enabled);

    void set_channel(byte channel);

    void rx_enable(void);
    void rx_disable(void);

    void rx_flush(void);

    rx_info_t * get_rxinfo(void);

    tx_info_t * get_txinfo(void);

    uint8_t * get_rxbuf(void);

    int rx_datalength(void);

    void set_ignoreBytes(int ib);

    void set_bufferPHY(boolean bp);

    boolean get_bufferPHY(void);

    void set_palna(boolean enabled);

    void send16(word dest16, char * data);

    void interrupt_handler(void);

    void check_flags(void (*rx_handler)(void), void (*tx_handler)(void));

