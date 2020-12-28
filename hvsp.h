#ifndef _HVSP_H_
#define _HVSP_H_

#include <util.h>

/* This will be used in OCRA value for clock generation
 * Should be multiple of 2 and 2 * val < 256, data timer will have double the period
 * This should be calculated offline
 * For F_CPU = 16000000 -> with 8 prescaler -> 2000000 / value = X khz
 * 16 seems to be too small, interrupt can't keep up
 */
#define PERIOD_CLOCK (28U)

#ifndef PERIOD_CLOCK
#error "NO PERIOD CLOCK"
#endif

/* Using those because of the timer output pin
 * Mandatory to be these pins on atmega324a
 */

#define SDI_PORT PORTD
#define SDI_DDR DDRD
#define SDI_PIN PD6

#define SII_PORT PORTD
#define SII_DDR DDRD
#define SII_PIN PD7

#define SCI_PORT PORTB
#define SCI_DDR DDRB
#define SCI_PIN PB3

/* End of section */

#define SDO_PORT PORTB
#define SDO_DDR DDRB
#define SDO_PIN PB1

/* Power and HV reset ports */

#define HV_PORT PORTB
#define HV_DDR DDRB
#define HV_PIN PB4

#define VCC_PORT PORTB
#define VCC_DDR DDRB
#define VCC_PIN PB5


PUBLIC void initHVSP(void);

/* Sending SDI and SII bytes to HVSP interface.
 * Not interrupt safe, not thread safe 
 */
PUBLIC void sendBytesHVSP(uint8_t sdi, uint8_t sii);
#endif