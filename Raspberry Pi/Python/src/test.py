import wiringpi
import time


wiringpi.wiringPiSetup() # wiringpi pin's number

pin_reset = 27
pin_cs = 26
pin_interrupt = 28

wiringpi.pinMode(pin_reset,1)
wiringpi.pinMode(pin_cs,1)
wiringpi.pinMode(pin_interrupt,0)

wiringpi.wiringPiSPISetup(0, 500000) # channel 0 (not used), 500 Khz / 2Mhz

wiringpi.digitalWrite(pin_reset,0)
time.sleep(0.02)
wiringpi.digitalWrite(pin_reset,1)
time.sleep(0.02)

address = 0x222

wiringpi.digitalWrite(pin_cs, 0)
ahigh = (address >> 3) | 0x80
alow = (address << 5) & 0xFF
bytesToSend = bytes([ahigh, alow, 0])
print(bytesToSend)
rx_tuple = wiringpi.wiringPiSPIDataRW(0, bytesToSend)
wiringpi.digitalWrite(pin_cs, 1)
print(rx_tuple)

#return rx_tuple[1][0]
