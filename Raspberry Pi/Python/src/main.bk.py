#!/usr/bin/env python

import time
import pigpio

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