#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>

#include <buffer.h>
#include <util.h>
#include <spi.h>
#include <dbg.h>
#include <timer.h>
#include <hvsp.h>

int main(void) {

    initHVSP();

    // Enable interrupts
    sei();

    uint8_t byte1 = 0x00;
    uint8_t byte2 = 0xFF;
    // Main loop
    while (1) {
        sendBytesHVSP(byte1, byte2);

        if (byte2 == 0x00) {
            byte2 = 0xff;
        } else {
            byte2--;
        }

        if (byte1 == 0xff) {
            byte1 = 0x00;
        } else {
            byte1++;
        }
    }

    return 0;
}