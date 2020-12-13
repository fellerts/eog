#include "uart.h"
#include "main.h"

// === CONSTANTS
const uint8_t Ts = 20;						// sample period (ms)
const uint8_t Bs = 1;						// baseline refresh period (minutes)
const uint8_t BUFFERLEN = 3;				// length of buffer => # samples to average over
const uint8_t REM_cycle_length = 15;		// length of delay after REM sleep is detected (minutes)
const uint8_t spikes_needed = 15;			// number of spikes needed for REM sleep to be true
const uint8_t spike_decrement_delay = 15;	// seconds between last spike and "spike_counter" decrementation
const uint8_t idle_delay = 60;				// how many idle minutes after boot before lucid induction

// === DATA VARIABLES
uint16_t data[6];							// data ring buffer
int16_t bufferPointer = 0;					// current start of data ring buffer
uint16_t max_amplitude;						// variable for keeping maximum amplitude

// === BASE LINE CALCULATION VARIABLES
uint16_t baseLine = 0;						// current base line
uint32_t temp_baseLine = 0;					// temp base line for updating
uint16_t Bs_cnt = 0;						// base line measure counter
uint16_t max_value, min_value;				// maximum and minimum values of current baseline calculation
bool unstable_base_line = false;			// true if values diverge too much
bool calculating_base_line = false;			// true when loop is calculating baseline parallel to sampling

// === SPIKE DETECTION VARIABLES
bool spike = false;							// true when spike is detected
bool REM = false;							// true when REM sleep is detected
uint16_t spike_value = 0;					// analog value of last spike
uint8_t spike_counter = 0;					// current number of spikes in cycle. if greater than 0, we are in a cycle.

// === TIMING VARIABLES
uint32_t last_baseline_timestamp;			// millis() at last baseline calculation
uint32_t last_sample_timestamp;				// millis() at last sample
uint32_t last_spike_event_timestamp;		// millis() at last spike event (increment or decrement), used in decrementation
uint32_t REM_cycle_start_timestamp;			// millis() at start of REM sleep cycle
uint32_t base_line_calculation_start;		// millis() at start of baseline calculation
volatile uint64_t millis_now;				// analogous to reading arduino millis()
bool idling;								// true while system is idling at start of night

int main(void){	
		
	DDRC |= (1 << STATUS_LED) | (1 << LED_STRIP); // outputs

	cli();	
	
	char buffer[100]; // buffer for converting integers to string
	buffer[99] = 0;
	
	init_adc();	// ADC init	
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); // UART init
	init_timer1(); // Timer1 init
		
	sei();
	
	// Let caps charge
	uart_puts("\r\nBooting. Charging caps..");
	uart_puts("\r\n");
	_delay_ms(1000);
	
	// Get baseline value
	while (baseLine == 0) {
		uart_puts("Measuring base line...");
		uart_puts("\r\n");
		baseLine = getBaseLine();
		
		if (baseLine == 0) {
			uart_puts("Keep still");
			uart_puts("\r\n");
		}
	}
	
	uart_puts("Base line: ");
	itoa(baseLine, buffer, 10);
	uart_puts(buffer);
	uart_puts("\r\n");
	
	// Get maximum gaze amplitude
	uart_puts("Go to extremities");
	uart_puts("\r\n");
	max_amplitude = getMaxAmplitude();
	uart_puts("A_max: ");
	itoa(max_amplitude, buffer, 10);
	uart_puts(buffer);
	
	uart_puts("; Pk tgt: ");
	itoa(max_amplitude / 4, buffer, 10);
	uart_puts(buffer);
	uart_puts("\r\n");

	uart_puts("System operational.");
	uart_puts("\r\n");
	uart_puts("Sleeping for ");
	itoa(idle_delay, buffer, 10);
	uart_puts(buffer);
	uart_puts(" minutes.");
	uart_puts("\r\n");
	_delay_ms(3000);
	
	// Initialize timer variables
	millis_now = 0;
	last_baseline_timestamp = 0;
	last_sample_timestamp = 0;
	last_spike_event_timestamp = 0;
	REM_cycle_start_timestamp = 0;
	idling = true;

	// Main loop start
	for(;;){

		// Time to sample?
		if ( (((uint32_t)millis_now - last_sample_timestamp) >= (uint32_t)Ts)) {

			// Timeout of current REM cycle?
			if (REM){
				if ((((uint32_t)millis_now - REM_cycle_start_timestamp) >= (uint32_t)60000 * REM_cycle_length)) {
					uart_puts("\r\nREM END\r\n");
					REM = false;
				}
			}

			// Sample
			last_sample_timestamp = millis_now;
			uint16_t sample = read_adc();
			pushBuffer(sample);
			int16_t mean = calculateMean(data);

			// Put sampled values on UART
			uart_puts("\r\n");
			itoa(mean, buffer, 10);
			uart_puts(buffer);

			// Is a spike detected?
			uint16_t abs_mean = abs(mean);
			if (abs_mean >= (max_amplitude / 4) ) {

				spike = true;
				
				// Hold highest value in spike
				if (abs_mean > spike_value) {
					spike_value = abs_mean;
				}
			}

			// Is there a spike, and has signal settled to a third of its maximum?
			if (spike) {
				if (abs_mean < (spike_value / 3) ) {
					
					// This marks the timestamp of the spike
					last_spike_event_timestamp = millis_now;

					// Increment spike counter, reset and ready for next spike
					spike_counter++;
					spike = false;
					spike_value = 0;
				}
			}
			
			// Time to decrement spike counter?
			if(spike_counter > 0){
				if ((((uint32_t)millis_now - last_spike_event_timestamp) >= (uint32_t)1000 * spike_decrement_delay)) {
					spike_counter--;
					last_spike_event_timestamp = millis_now;
				}
			}
			
			// Time to stop idling and start lucid induction?
			if(idling){
				if (((uint32_t)millis_now >= (uint32_t)60000 * idle_delay)) {
					idling = false;
					spike_counter = 0; // Also reset spike counter for a fresh start
				}
			}
			
			// REM sleep detected?
			if ( !REM & !idling ) {
				if (spike_counter >= spikes_needed) {
					REM_cycle_start_timestamp = millis_now;
					REM = true;
					uart_puts("\r\nREM START, FLASHING\r\n");
					flash_rapid();
				}
			}

			// Time to start calculating base line?
			if (!calculating_base_line){
				if ((((uint32_t)millis_now - last_baseline_timestamp) >= (uint32_t)60000 * Bs)) {
					
					uart_puts("Calculating baseline..");
					uart_puts("\r\n");
					calculating_base_line = true;
					base_line_calculation_start = millis_now;
					Bs_cnt = 0;
					temp_baseLine = 0;
					max_value = sample;
					min_value = sample;
					unstable_base_line = false;
				}
			}

			// If we are calculating base line...
			if (calculating_base_line) {

				// Are we done calculating base line?
				if (((uint32_t)millis_now - base_line_calculation_start) >= (uint32_t)3000) {
					uart_puts("Done!");
					uart_puts("\r\n");
					
					calculating_base_line = false;
					last_baseline_timestamp = millis_now;
					baseLine = temp_baseLine / Bs_cnt;
				}
				else {
					// If no, read for baseline..
					if (sample > max_value) {
						max_value = sample;
					}
					else if (sample < min_value) {
						min_value = sample;
					}

					// If spread in measurements is larger than 20 we have no basis for baseline
					if ((max_value - min_value) > 20) {
						unstable_base_line = true;
						last_baseline_timestamp = millis_now;
						calculating_base_line = false;
					}
					else {
						temp_baseLine += sample;
						Bs_cnt++;
					}
				}
			}

			// Update status LED with spike and REM information
			if( REM | spike){
				PORTC |= (1 << STATUS_LED);
			}else{
				PORTC &= ~(1 << STATUS_LED);
			}

			// Print spike/REM status to serial port
			uart_puts("\t");
			itoa(spike_counter, buffer, 10);
			uart_puts(buffer);

		} //end if(time to sample)
	} // end main loop
} // end main

uint16_t read_adc(){
	uint16_t avg = 0;
	ADMUX |= (1 << MUX2) | (1 << MUX1) | (1 << MUX0); // Select ADC7 as active channel
	
	for(uint8_t i = 0; i < 8; i++){	
		ADCSRA |= (1 << ADSC); // start conversion
		while(!(ADCSRA & (1 << ADIF)) ); // wait for conversion to complete
		ADCSRA |= (1 << ADIF); // clear interrupt flag (by writing one to it as per datasheet)
		avg += ADC; // adds value
		_delay_us(10);
	}
	
	return (avg / 8);
}

void init_adc(){
	 ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 62.5 kHz sample rate @ 8 MHz
	 ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	 ADCSRA |= (1 << ADEN);  // Enable ADC
}

void init_timer1(){
	OCR1A = 1000; // compare match value for 1 ms @ 8 MHz / 8
	TCCR1B |= (1 << WGM12);
	TCCR1B |= (1 << CS11);// initialize timer1 with 8 prescaler
	TIMSK1 |= (1 << OCIE1A);
	TCNT1 = 0; // initialize counter
}

// pushes value to buffer
void pushBuffer(uint16_t val) {
	--bufferPointer;

	if (bufferPointer < 0) {
		bufferPointer = BUFFERLEN - 1;		// loop back if underflow
	}

	data[bufferPointer] = val;				// update with current sample
}

// returns mean of "filterSamples" samples, adjusted with baseline
int16_t calculateMean(uint16_t data[]) {

	int32_t mean = 0;
	for (uint8_t i = 0; i < BUFFERLEN; ++i) {
		uint8_t pos = (bufferPointer + i) % BUFFERLEN;

		if (data[pos] == 0) {
			continue; 						// uninitialized value, don't consider
		}

		mean += (int32_t)data[pos] - baseLine;
	}

	return mean/BUFFERLEN;
}

// calculates baseline for DC offset compensation over 3-ish seconds
uint16_t getBaseLine() {
	int32_t base, current, min, max;
	base = 0;

	// let signal settle at baseline
	_delay_ms(1000);

	// initial read
	current = read_adc();
	min = current;
	max = current;

	// baseline read at 10 ms intervals
	for (uint16_t i = 0; i < 300; i++) {
		current = read_adc();

		if (current < min) {
			min = current;
		}
		else if (current > max) {
			max = current;
		}

		// if spread in measurements is larger than 20 we have no basis for baseline
		if ((max - min) > 20) {
			return baseLine;
		}

		base += current;
		_delay_ms(10);
	}

	// integer division to avoid floating point operations
	base /= 300;

	// off-center baseline indicates electrode problem; die
	if (base < 200 || base > 800) {
		uart_puts("Skewed baseline, dying..");
		uart_puts("\r\n");
		die();
	}
	return base;
}

// calculates maximum amplitude at beginning of cycle over 5-ish seconds
uint32_t getMaxAmplitude() {

	int32_t max = 0, current;

	for (int i = 0; i < 500; i++) {
		current = read_adc() - (int32_t)baseLine;
		if (abs(current) > max) {
			max = abs(current);
		}

		_delay_ms(10);
	}

	if (max > 1000) {
		uart_puts("Too large maximum: dying");
		uart_puts("\r\n");
		die();
	}

	return max;
}

// flashes REM lights
void flash_rapid() {	
	for(uint8_t i = 0; i < 10; i++){
		flash_once();
	}
}

// flashes REM lights once with delays gives as arguments
void flash_once(){
	PORTC |= (1 << LED_STRIP);
	_delay_ms(200);
	PORTC &= ~(1 << LED_STRIP);
	_delay_ms(500);
}

// horrible function of death
void die() {
	for (;;);
}

ISR (TIMER1_COMPA_vect){
	++millis_now;
}

/*	Change log since 10.05.2016
		- increased duty cycle of flash_rapid() from 30/400 to 200/500
	Change log since 11.05.2016
		- increased decrementation delay to 15 s
		- decreased REM cycle length to 15 minutes
*/