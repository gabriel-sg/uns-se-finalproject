/**
 * Example code for using a microchip mrf24j40 module to send and receive
 * packets using plain 802.15.4
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
 */
#include <mrf24j_wiringPi.h>
#include <wiringPi.h>
#include <stdio.h>

void interrupt_routine();
void handle_rx();
void handle_tx();

const int pin_reset = 27;
const int pin_cs = 26;         // default CS pin on ATmega8/168/328
const int pin_interrupt = 28;  // default interrupt pin on ATmega8/168/328
const int spi_channel = 0;

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt, spi_channel);

long last_time;
long tx_interval = 2000;

int main() {
    mrf.reset();
    mrf.init();

    mrf.set_pan(0xcafe);
    // This is _our_ address
    mrf.address16_write(0x6001);

    // uncomment if you want to receive any packet on this channel
    mrf.set_promiscuous(false);

    // uncomment if you want to enable PA/LNA external control
    mrf.set_palna(true);

    mrf.set_channel(18);

    // uncomment if you want to buffer all PHY Payload
    //mrf.set_bufferPHY(true);

    // Set interrupt edge rising. (default = falling)
    //mrf.write_long(MRF_SLPCON0, 0b00000010);

    wiringPiISR (pin_interrupt, INT_EDGE_FALLING, &interrupt_routine) ;

    last_time = millis();
    while(1){
        mrf.check_flags(&handle_rx, &handle_tx);
        unsigned long current_time = millis();
        if (current_time - last_time > tx_interval) {
            last_time = current_time;
            printf("txxxing...\n");
            mrf.send16(0x6005, "abcd");
            // delay(1000);
            // mrf.send16(0x6003, "pepe");
        }
    }
    return 0;
}

void interrupt_routine() {
    mrf.interrupt_handler();  // mrf24 object interrupt routine
}


void handle_rx() {
    printf("received a packet ");
    printf("%i",mrf.get_rxinfo()->frame_length);
    printf(" bytes long\n");

    if (mrf.get_bufferPHY()) {
        printf("Packet data (PHY Payload):\n");
        for (int i = 0; i < mrf.get_rxinfo()->frame_length; i++) {
            printf("%i",mrf.get_rxbuf()[i]);
        }
    }

    printf("\r\nASCII data (relevant data):\n");
    for (int i = 0; i < mrf.rx_datalength(); i++) {
        //Serial.write(mrf.get_rxinfo()->rx_data[i]);
        printf("%c", mrf.get_rxinfo()->rx_data[i]);
    }

    printf("\r\nLQI/RSSI=");
    printf("%i",mrf.get_rxinfo()->lqi);
    printf("/");
    printf("%i\n",mrf.get_rxinfo()->rssi);
}

void handle_tx() {
    if (mrf.get_txinfo()->tx_ok) {
        printf("TX went ok, got ack\n");
    } else {
        printf("TX failed after ");
        printf("%i",mrf.get_txinfo()->retries);
        printf(" retries\n\n");
    }
}
