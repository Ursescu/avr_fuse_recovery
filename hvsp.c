#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include <util.h>
#include <dbg.h>

#include <hvsp.h>

#define NUM_BITS(var) (sizeof((var)) * 8U)

typedef enum {
    IDLE = 0x00,
    START,
    START_STAGE1,
    TX,
    FINISH
} HVP_state;

PRIVATE volatile HVP_state state = IDLE;
PRIVATE volatile uint8_t sdi;
PRIVATE volatile uint8_t sii;
PRIVATE volatile uint8_t sdo;
PRIVATE volatile uint8_t shift;

PRIVATE void toggleSCI(void) __attribute__((always_inline));
PRIVATE void setLowSCI(void) __attribute__((always_inline));
PRIVATE void setHighSCI(void) __attribute__((always_inline));
PRIVATE void enableSCITimer(void) __attribute__((always_inline));
PRIVATE void disableSCITimer(void) __attribute__((always_inline));

PRIVATE void toggleSDI(void) __attribute__((always_inline));
PRIVATE void setLowSDI(void) __attribute__((always_inline));
PRIVATE void setHighSDI(void) __attribute__((always_inline));
PRIVATE void setLowSDITimer(void) __attribute__((always_inline));
PRIVATE void setHighSDITimer(void) __attribute__((always_inline));

PRIVATE void setLowSII(void) __attribute__((always_inline));
PRIVATE void setHighSII(void) __attribute__((always_inline));
PRIVATE void toggleSII(void) __attribute__((always_inline));
PRIVATE void setLowSIITimer(void) __attribute__((always_inline));
PRIVATE void setHighSIITimer(void) __attribute__((always_inline));

PRIVATE void enableDataTimer(void) __attribute__((always_inline));
PRIVATE void disableDataTimer(void) __attribute__((always_inline));

PRIVATE void initVCCPin(void) __attribute__((always_inline));
PRIVATE void enableVCCPin(void) __attribute__((always_inline));
PRIVATE void disableVCCPin(void) __attribute__((always_inline));

PRIVATE void disableHVPin(void) __attribute__((always_inline));
PRIVATE void enableHVPin(void) __attribute__((always_inline));
PRIVATE void initHVPin(void) __attribute__((always_inline));

/* Transform least significat bit to most significat bit 8 bit variable */
PRIVATE uint8_t lsbToMsb(uint8_t data);

/* Function that will prepare the timer options for the next SII bit */
PRIVATE void prepareNextSIIBit(void);

/* Function that will prepare the timer options for the next SDI bit */
PRIVATE void prepareNextSDIBit(void);

/* Starting a HVSP transaction */
PRIVATE void startTransaction(void);

// Init HVSP
PUBLIC void initHVSP(void) {
    // CTC on OCR0A - also toggle OCC0A port
    TCCR0A |= _BV(WGM01);
    // Enable OCR0A interrupts
    TIMSK0 |= _BV(OCIE0A);

    // First time should be half in order to start the second timer
    OCR0A = PERIOD_CLOCK / 2;

    // Timer 2 clocked by the first timer
    TCCR2A |= _BV(WGM21);

    TIMSK2 |= _BV(OCIE2A) | _BV(OCIE2B);

    OCR2A = PERIOD_CLOCK * 2;
    OCR2B = PERIOD_CLOCK * 2;

    // Output
    SDI_DDR |= _BV(SDI_PIN);
    SII_DDR |= _BV(SII_PIN);
    SCI_DDR |= _BV(SCI_PIN);

    // Input - but need as output for entering programming mode
    SDO_DDR |= _BV(SDO_PIN);

    // Set them on low
    setLowSCI();
    setLowSDI();
    setLowSII();

    SDO_PORT &= ~(_BV(SDO_PIN));

    initHVPin();
    disableHVPin();

    initVCCPin();
    disableVCCPin();
}

PUBLIC void enterHVSPMode(void) {
    // Set them on low
    setLowSDI();
    setLowSII();

    // Configure SDO as output and set it low
    SDO_DDR |= _BV(SDO_PIN);
    SDO_PORT &= ~(_BV(SDO_PIN));

    // Disable HV pin and VCC pin
    disableHVPin();
    disableVCCPin();

    _delay_ms(20);

    // Start the procedure

    enableVCCPin();
    _delay_us(20);
    enableHVPin();
    _delay_us(10);

    // Release it as input
    SDO_DDR &= ~(_BV(SDO_PIN));

    _delay_us(300);
}

PUBLIC uint8_t sendBytesHVSP(uint8_t siiByte, uint8_t sdiByte) {
    sdi = lsbToMsb(sdiByte);
    sii = lsbToMsb(siiByte);
    sdo = 0U;
    startTransaction();

    // Block here until the end
    while (state != IDLE) {
        ;
    }

    return sdo;
}

/* Clock interrupt, both pos and negative edge */
ISR(TIMER0_COMPA_vect) {
    switch (state) {
        case START: {
            /* Wait for one more half period in order to start aligned */
            state = START_STAGE1;

            enableDataTimer();
            
            TCCR0A &= ~(_BV(COM0A1));
            TCCR0A |= _BV(COM0A0);

            break;
        }
        case START_STAGE1: {
            state = TX;

            OCR0A = PERIOD_CLOCK;
            break;
        }
        case TX: {
            // Reading bit from target only on falling edge
            if ((shift > 5U) && (shift % 2 == 0U)) {
                sdo |= !!(PINB & (_BV(SDO_PIN)));
                if (shift > 7U) {
                    sdo <<= 1;
                }
            }
            // Finished transmitting the byte
            if (!shift) {
                state = FINISH;

                disableSCITimer();
                setLowSIITimer();
                setLowSDITimer();
            } else {
                shift--;
            }
            break;
        }
        default:
            /* No action */
            break;
    }
}

/* Interrupt for SII */
ISR(TIMER2_COMPA_vect) {
    if (state != FINISH) {
        prepareNextSIIBit();
    } else {
        state = IDLE;
        disableDataTimer();
    }
}

/* Interrupt for SDI */
ISR(TIMER2_COMPB_vect) {
    prepareNextSDIBit();
}

PRIVATE uint8_t lsbToMsb(uint8_t data) {
    uint8_t temp = data & 0x01;
    uint8_t numbits = NUM_BITS(data) - 1;

    for (data >>= 1; data; data >>= 1) {
        temp <<= 1;
        temp |= data & 1;
        numbits--;
    }
    temp <<= numbits;

    return temp;
}

PRIVATE void prepareNextSIIBit(void) {
    if (sii & 0x01) {
        // Should set it
        setHighSIITimer();
    } else {
        // Should clear it
        setLowSIITimer();
    }

    sii >>= 1;
}

PRIVATE void prepareNextSDIBit(void) {
    if (sdi & 0x01) {
        // Should set it
        setHighSDITimer();
    } else {
        // Should clear it
        setLowSDITimer();
    }

    sdi >>= 1;
}

PRIVATE void startTransaction(void) {
    // Start by setting state
    state = START;

    // Ensure a clean state for the timers
    TCNT0 = 0U;
    TCNT2 = 0U;

    TIFR0 = 0U;
    TIFR2 = 0U;

    TCCR0A &= ~(_BV(COM0A0) | _BV(COM0A1));

    OCR0A = PERIOD_CLOCK / 2U;

    // Counted on both pos neg edges of clock
    shift = NUM_BITS(sdi) * 2U + 4;

    // Prepare the first bits
    prepareNextSDIBit();
    prepareNextSIIBit();

    // Start the machine
    enableSCITimer();
}

/* Accesors to different parts of AVR timers */

/* SCI */
PRIVATE void toggleSCI(void) {
    SCI_PORT ^= _BV(SCI_PIN);
}

PRIVATE void setLowSCI(void) {
    SCI_PORT &= ~(_BV(SCI_PIN));
}

PRIVATE void setHighSCI(void) {
    SCI_PORT |= _BV(SCI_PIN);
}

PRIVATE void enableSCITimer(void) {
    TCCR0B |= _BV(CS01);
}

PRIVATE void disableSCITimer(void) {
    TCCR0B &= ~(_BV(CS01));
}
/* End SCI */

/* SDI */
PRIVATE void setLowSDI(void) {
    SDI_PORT &= ~(_BV(SDI_PIN));
}

PRIVATE void setHighSDI(void) {
    SDI_PORT |= _BV(SDI_PIN);
}

PRIVATE void setLowSDITimer(void) {
    TCCR2A |= _BV(COM2A1);
    TCCR2A &= ~(_BV(COM2A0));
}

PRIVATE void setHighSDITimer(void) {
    TCCR2A |= _BV(COM2A0) | _BV(COM2A1);
}

PRIVATE void toggleSDI(void) {
    SDI_PORT ^= _BV(SDI_PIN);
}
/* End SDI */

/* SII */
PRIVATE void setLowSII(void) {
    SII_PORT &= ~(_BV(SII_PIN));
}

PRIVATE void setHighSII(void) {
    SII_PORT |= _BV(SII_PIN);
}

PRIVATE void setLowSIITimer(void) {
    TCCR2A |= _BV(COM2B1);
    TCCR2A &= ~(_BV(COM2B0));
}

PRIVATE void setHighSIITimer(void) {
    TCCR2A |= _BV(COM2B0) | _BV(COM2B1);
}

PRIVATE void toggleSII(void) {
    SII_PORT ^= _BV(SII_PIN);
}
/* End SII */

/* Data timer */
PRIVATE void enableDataTimer(void) {
    TCCR2B |= _BV(CS21);
}

PRIVATE void disableDataTimer(void) {
    TCCR2B &= ~(_BV(CS21));
}
/* End Data timer */

/* HV Reset */
PRIVATE void disableHVPin(void) {
    HV_PORT |= _BV(HV_PIN);
}

PRIVATE void enableHVPin(void) {
    HV_PORT &= ~(_BV(HV_PIN));
}

PRIVATE void initHVPin(void) {
    HV_DDR |= _BV(HV_PIN);
}
/* End HV */

/* VCC */
PRIVATE void disableVCCPin(void) {
    VCC_PORT &= ~(_BV(VCC_PIN));
}

PRIVATE void enableVCCPin(void) {
    VCC_PORT |= _BV(VCC_PIN);
}

PRIVATE void initVCCPin(void) {
    VCC_DDR |= _BV(VCC_PIN);
}
/* End VCC */
