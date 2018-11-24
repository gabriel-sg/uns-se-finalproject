#!/usr/bin/env python

import time
import pigpio

# spi_open(spi_channel, baud, spi_flags)
# pi.spi_open(0, 1000000, 0)   # CE0, 1Mbps, main SPI
# pi.spi_open(1, 1000000, 0)   # CE1, 1Mbps, main SPI
# pi.spi_open(0, 1000000, 256) # CE0, 1Mbps, auxiliary SPI
# pi.spi_open(1, 1000000, 256) # CE1, 1Mbps, auxiliary SPI
# pi.spi_open(2, 1000000, 256) # CE2, 1Mbps, auxiliary SPI

# spi_flags consists of the least significant 22 bits.
# 21 20 19 18 17 16 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
#  b  b  b  b  b  b  R  T  n  n  n  n  W  A u2 u1 u0 p2 p1 p0  m  m

# mm defines the SPI mode. 0 0 = mode 0
# px is 0 if CEx is active low (default) and 1 for active high.
# ux is 0 if the CEx GPIO is reserved for SPI (default) and 1 otherwise.
# A is 0 for the main SPI, 1 for the auxiliary SPI.
# W is 0 if the device is not 3-wire, 1 if the device is 3-wire. Main SPI only.
# nnnn defines the number of bytes (0-15) to write before switching the MOSI line to MISO to read data. This field is ignored if W is not set. Main SPI only.
# T is 1 if the least significant bit is transmitted on MOSI first, the default (0) shifts the most significant bit out first. Auxiliary SPI only.
# R is 1 if the least significant bit is received on MISO first, the default (0) receives the most significant bit first. Auxiliary SPI only.
# bbbbbb defines the word size in bits (0-32). The default (0) sets 8 bits per word. Auxiliary SPI only.

PIN_CS = 12
PIN_RESET = 16
PIN_INTERRUPT = 20
SPI_SPEED = 500000

pi = pigpio.pi()
if not pi.connected:
    exit(0)

def spi_transfer(byte):
    byte = byte & 255 # truncado a 8 bits
    rx_tuple = pi.spi_xfer(spi_handler, [byte])
    return rx_tuple[1][0]

def read_short(address):
    pi.write(PIN_CS, pigpio.LOW)
    spi_transfer(address << 1 & 0b01111110)
    ret = spi_transfer(0)
    pi.write(PIN_CS, pigpio.HIGH)
    return ret

def read_long(address):
    pi.write(PIN_CS, pigpio.LOW)
    ahigh = address >> 3
    alow = address << 5
    spi_transfer(0x80 | ahigh)
    spi_transfer(alow)
    ret = spi_transfer(0)
    pi.write(PIN_CS, pigpio.HIGH)
    return ret


def write_short(address, data):
    pi.write(PIN_CS, pigpio.LOW)
    spi_transfer((address<<1 & 0b01111110) | 0x01)
    spi_transfer(data)
    pi.write(PIN_CS, pigpio.HIGH)

def write_long(address, data):
    pi.write(PIN_CS, pigpio.LOW)
    ahigh = address >> 3
    alow = address << 5
    spi_transfer(0x80 | ahigh)
    spi_transfer(alow | 0x10)
    spi_transfer(data)
    pi.write(PIN_CS, pigpio.HIGH)


# Reset MRF
pi.set_mode(PIN_RESET, pigpio.OUTPUT)
pi.write(PIN_RESET, pigpio.LOW)
time.sleep(0.010)  # 10ms
pi.write(PIN_RESET, pigpio.HIGH)
time.sleep(0.020)

pi.set_mode(PIN_CS, pigpio.OUTPUT)

spi_handler = pi.spi_open(0, 500000, 0)


send_byte = 0x2E
read_byte = read_short(send_byte)
print("send value: {}, read value: {}".format(hex(send_byte), hex(read_byte)))

write_value = 0x95
send_byte = 0x2E
write_short(send_byte, write_value)
read_byte = read_short(send_byte)
print("write value: {}, read value: {}".format(hex(write_value), hex(read_byte)))

send_byte = 0x220
read_byte = read_long(send_byte)
print("send value: {}, read value: {}".format(hex(send_byte), hex(read_byte)))

# contWriteSucc += assertWriteLong("MRF_SLPCON1", MRF_SLPCON1, 0x21, printResults);

write_value = 0x21
send_byte = 0x220
write_long(send_byte, write_value)
read_byte = read_long(send_byte)
print("write value: {}, read value: {}".format(hex(write_value), hex(read_byte)))

pi.spi_close(spi_handler)

pi.stop()
