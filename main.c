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

PRIVATE void readFuseLow() {
    uint8_t ret = 0U;

    // Read fuses low byte
    ret = sendBytesHVSP(0b01001100, 0b00000100);
    _delay_ms(1);
    ret = sendBytesHVSP(0b01101000, 0x00);
    _delay_ms(1);
    ret = sendBytesHVSP(0b01101100, 0x00);
    printfDBG("0x%02x\n", ret);
    _delay_ms(1);
}

PRIVATE void readFuseHigh() {
    uint8_t ret = 0U;

    // Read fuses high byte
    ret = sendBytesHVSP(0b01001100, 0b00000100);
    _delay_ms(1);
    ret = sendBytesHVSP(0b01111010, 0x00);
    _delay_ms(1);
    ret = sendBytesHVSP(0b01111110, 0x00);
    printfDBG("0x%02x\n", ret);
    _delay_ms(1);
}

PRIVATE void readFuseEx() {
    uint8_t ret = 0U;

    // Read fuses extended
    ret = sendBytesHVSP(0b01001100, 0b00000100);
    _delay_ms(1);
    ret = sendBytesHVSP(0b01101010, 0x00);
    _delay_ms(1);
    ret = sendBytesHVSP(0b01101110, 0x00);
    printfDBG("0x%02x\n", ret);
    _delay_ms(1);
}

int main(void) {
    initDBG();

    initHVSP();

    // Enable interrupts
    sei();

    enterHVSPMode();

    readFuseLow();
    readFuseHigh();
    readFuseEx();
    uint8_t ret;

    // Write high fuse
    ret = sendBytesHVSP(0b01001100, 0b01000000);
    _delay_ms(1);

    ret = sendBytesHVSP(0b00101100, 0xdd);
    _delay_ms(1);

    ret = sendBytesHVSP(0b01110100, 0x00);
    _delay_ms(1);

    ret = sendBytesHVSP(0b01111100, 0x00);
    _delay_ms(1);

    _delay_ms(20);

    readFuseLow();
    readFuseHigh();
    readFuseEx();

    // Main loop
    while (1) {
        ;
    }

    return 0;
}