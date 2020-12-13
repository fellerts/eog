//#ifndef MAIN_H_
//#define MAIN_H_

#define F_CPU				8000000
#define UART_BAUD_RATE      57600
#define	true				1
#define	false				0
#define	STATUS_LED			PC4
#define	LED_STRIP			PC5

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

typedef uint8_t bool;

// calls all setup functions and preps for sampling
void setup();

// initializes ADC
void init_adc();

// initializes Timer1 for CTC mode
void init_timer1();

// reads ADC, averages over 8 samples
uint16_t read_adc();

// calculates baseline for DC offset compensation over 3-ish seconds
uint16_t getBaseLine();

// calculates maximum amplitude at beginning of cycle over 5-ish seconds
uint32_t getMaxAmplitude();

// flashes status led rapidly
void flash_rapid();

// flashes REM lights once with delays gives as arguments
void flash_once();

// horrible function of death
void die();

// pushes value to buffer
void pushBuffer(uint16_t val);

//#endif /* MAIN_H_ */