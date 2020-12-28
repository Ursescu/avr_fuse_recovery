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

#define UART_BAUDRATE 38400

int main(void) {
    initMSTimer();

    // Enable debug print
    initDBG();

    // Init SPI, mode 0 with clock div 128
    initSPI(0, 3);

    // Debug led output
    DDRD |= _BV(PD7);

    // Enable interrupts
    sei();

    // Main loop
    while (1) {
        PORTD ^= _BV(PD7);
        printfDBG("Hello\n");
        delay_ms(500);
    }

    return 0;
}