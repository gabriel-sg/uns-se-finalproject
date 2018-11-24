#!/usr/bin/env python

import mfr24j40
import pigpio

pin_reset = 16
pin_cs = 12
pin_interrupt = 20
spi_channel = 0


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

def interrupt_routine():
    mrf.interrupt_handler()

pi.callback(pin_interrupt,pigpio.FALLING_EDGE,interrupt_routine)

#last_time = millis()
while True:
    #mrf.check_flags(&handle_rx, &handle_tx)
    print("PANId: {} - Addr: {}".format(hex(mrf.get_pan()),hex(mrf.address16_read())))


# void handle_rx() {
#     printf("received a packet ");
#     printf("%i",mrf.get_rxinfo()->frame_length);
#     printf(" bytes long\n");

#     if (mrf.get_bufferPHY()) {
#         printf("Packet data (PHY Payload):\n");
#         for (int i = 0; i < mrf.get_rxinfo()->frame_length; i++) {
#             printf("%i",mrf.get_rxbuf()[i]);
#         }
#     }

#     printf("\r\nASCII data (relevant data):\n");
#     for (int i = 0; i < mrf.rx_datalength(); i++) {
#         //Serial.write(mrf.get_rxinfo()->rx_data[i]);
#         printf("%c", mrf.get_rxinfo()->rx_data[i]);
#     }

#     printf("\r\nLQI/RSSI=");
#     printf("%i",mrf.get_rxinfo()->lqi);
#     printf("/");
#     printf("%i\n",mrf.get_rxinfo()->rssi);
# }

# void handle_tx() {
#     if (mrf.get_txinfo()->tx_ok) {
#         printf("TX went ok, got ack\n");
#     } else {
#         printf("TX failed after ");
#         printf("%i",mrf.get_txinfo()->retries);
#         printf(" retries\n\n");
#     }
# }
