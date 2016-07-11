#define F_CPU 9600000UL
#define VOLTAGE_MON			// Comment out to disable - ramp down and eventual shutoff when battery is low
#define TURBO				// Comment out to disable - full output with a step down after n number of seconds
#define TURBO_TIMEOUT	5625 // How many WTD ticks before before dropping down (.016 sec each)  // 90  = 5625  // 120 = 7500
#define ADC_LOW			124	// When do we start ramping
#define ADC_CRIT		124 // When do we shut the light off
							// nanjg value = (V * 4700 * 255) / (23800 * 1.1) = (V * 45,78)
							// my value = (V * 46.14)
#define ADC_DELAY		122	// Delay in ticks between low-bat rampdowns (188 ~= 3s)
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>	
#include <avr/eeprom.h>
#include <avr/sleep.h>
#define SWITCH_PIN  PB3		// what pin the switch is connected to, which is Pin2 (Star 4)
#define VOLTAGE_PIN PB2		// Pin7
#define ADC_CHANNEL 0x01	// MUX 01 corresponds with PB2
#define ADC_DIDR 	ADC1D	// Digital input disable bit corresponding with PB2
#define ADC_PRSCL   0x06	// clk/64
#define Pin_5		PB0		// defines a name for PB0, the port at Pin5; let's name it... Pin_5
#define Pin_5_PWM	OCR0A	// defines a name for OCR0A, the output compare register for PB0 (which sets the PWM value)
#define Pin_6		PB1		// defines a name for PB1, which is the port at Pin6
#define Pin_6_PWM   OCR0B	// defines a name for OCR0B, the output compare register for PB1 (which sets the PWM value)
#define Pin_3		PB4		// defines a name for PB4, which is the port at Pin3

uint8_t highest_mode = 5;	// #################### CHECK THIS !!!!!!! ############################################

// #define DB_PRES_DUR 0b00000001	// time before we consider the switch pressed (after first realizing it was pressed)
#define DB_REL_DUR  0b00001111		// time before consider released. Each bit of 1 from the right = 16ms, so 0x0f = 64ms
#define LONG_PRESS_DUR  40		// How many WDT ticks until we consider a press a long press	// 32 is roughly .5 s
#define VERY_LONG_PRESS_DUR  96

volatile uint8_t mode_idx = 0;
volatile uint8_t press_duration = 0;

void set_output() {

			if (mode_idx == 0) {		// **** Turn Off ****
			TCCR0A = 0b00000001;		// no PWM
			PORTB &= ~(1 << Pin_5);		// OFF
			PORTB &= ~(1 << Pin_6);		// OFF
			PORTB &= ~(1 << Pin_3);		// OFF
			}
			if (mode_idx == 1) {		// **** MOON ****
			TCCR0A = 0b10000001;		// PhC PWM on Pin_5, no PWM on Pin_6
			Pin_5_PWM = 2;				// PWM level 3/255 for Pin_5
			PORTB &= ~(1 << Pin_6);		// OFF
			PORTB &= ~(1 << Pin_3);		// OFF
			}
			if (mode_idx == 2) {		// **** 35 mA ****
			TCCR0A = 0b10000001;		// PhC PWM on Pin_5, no PWM on Pin_6
			Pin_5_PWM = 34;				// PWM level for Pin_5
			PORTB &= ~(1 << Pin_6);		// OFF
			PORTB &= ~(1 << Pin_3);		// OFF
			}
			if (mode_idx == 3) {		// **** 350 mA ****
			TCCR0A = 0b00000001;		// no PWM
			PORTB |= (1 << Pin_5);		// *ON*
			PORTB &= ~(1 << Pin_6);		// OFF
			PORTB &= ~(1 << Pin_3);		// OFF
			}
			if (mode_idx == 4) {		// **** HIGH ****
			TCCR0A = 0b00000001;		// no PWM
			PORTB |= (1 << Pin_5);		// *ON*
			PORTB |= (1 << Pin_6);		// *ON*
			PORTB &= ~(1 << Pin_3);		// OFF
			}
			if (mode_idx == 5) {		// **** TURBO ****
			TCCR0A = 0b00000001;		// no PWM
			PORTB &= ~(1 << Pin_5);		// OFF
			PORTB &= ~(1 << Pin_6);		// OFF
			PORTB |= (1 << Pin_3);		// *ON*
			}
	}

int is_pressed()
{
	static uint8_t buffer = 0x00;	// Keep track of last switch values polled
	// Shift over and tack on the latest value, 0 being low for pressed, 1 for pulled-up for released
	buffer = (buffer << 1) | ((PINB & (1 << SWITCH_PIN)) == 0);
	return (buffer & DB_REL_DUR);
}

inline void next_mode() {
	if (++mode_idx > highest_mode) {	// NO Wrap around
		mode_idx = highest_mode;
	}	
}

inline void prev_mode() {
	if (mode_idx >= 1) {	// NO Wrap around
		--mode_idx;
	}
}

inline void PCINT_on() {	// Enable pin change interrupts
	GIMSK |= (1 << PCIE);
}

inline void PCINT_off() {	// Disable pin change interrupts
	GIMSK &= ~(1 << PCIE);
}

EMPTY_INTERRUPT(PCINT0_vect);	// Need an interrupt for when pin change is enabled to ONLY wake us from sleep.
								// All logic of what to do when we wake up will be handled in the main loop.
inline void WDT_on() {	// Setup watchdog timer to only interrupt, not reset, every 16ms.
	cli();							// Disable interrupts
	wdt_reset();					// Reset the WDT
	WDTCR |= (1<<WDCE) | (1<<WDE);  // Start timed sequence
	WDTCR = (1<<WDTIE);				// Enable interrupt every 16ms
	sei();							// Enable interrupts
}

inline void WDT_off() {
	cli();							// Disable interrupts
	wdt_reset();					// Reset the WDT
	MCUSR &= ~(1<<WDRF);			// Clear Watchdog reset flag
	WDTCR |= (1<<WDCE) | (1<<WDE);  // Start timed sequence
	WDTCR = 0x00;					// Disable WDT
	sei();							// Enable interrupts
}

inline void ADC_on() {
	ADMUX  = (1 << REFS0) | (1 << ADLAR) | ADC_CHANNEL; // 1.1v reference, left-adjust, ADC1/PB2
    DIDR0 |= (1 << ADC_DIDR);	// disable digital input on ADC pin to reduce power consumption
	ADCSRA = (1 << ADEN ) | (1 << ADSC ) | ADC_PRSCL;   // enable, start, prescale
}

inline void ADC_off() {
	ADCSRA &= ~(1<<7); //ADC off
}

void sleep_until_switch_press() {
	WDT_off();	// Turn the WDT off so it doesn't wake us from sleep
				// Will also ensure interrupts are on or we will never wake up		
	press_duration = 0;	// Need to reset press duration since a button release wasn't recorded
	while (is_pressed()) {	// Enable a pin change interrupt to wake us up. However, we have
		_delay_ms(16);		// to make sure the switch is released or we will wake when the user releases it
		}
	PCINT_on();
	sleep_mode();	// Now go to sleep
			// Hey, someone must have pressed the switch!!
	PCINT_off();	// Disable pin change interrupt because it's only used to wake us up
	WDT_on();	// Turn the WDT back on to check for switch presses
}				// Go back to main program

ISR(WDT_vect) {		// The watchdog timer is called every 16ms
	static uint16_t turbo_ticks = 0;
	static uint8_t  adc_ticks = ADC_DELAY;
	static uint8_t  lowbatt_cnt = 0;

	if (is_pressed()) {
		if (press_duration < 255) {
			press_duration++;
		}
		
		if (press_duration == LONG_PRESS_DUR && mode_idx >= 2) {		// LongPress in Low or higher
			prev_mode();												// Mode -1
		}																// (NO mode decrease from Moon, only VeryLongPress turns Off)

		if (press_duration == VERY_LONG_PRESS_DUR && mode_idx == 0) {	// VeryLong press in Off
			mode_idx = highest_mode;									// Turbo
			}

		if (press_duration == VERY_LONG_PRESS_DUR && mode_idx > 0 && mode_idx < highest_mode) {	// VeryLong press in Moon...High
			mode_idx = 0;																		// Off
			}																					// (NO Off directly from Turbo, or previous command has no effect)

		turbo_ticks = 0;	// Just always reset turbo timer whenever the button is pressed
		adc_ticks = ADC_DELAY;	// Same with the ramp down delay
	
	} else {	// Not pressed
		
		if (press_duration > 0 && press_duration < LONG_PRESS_DUR && mode_idx != highest_mode) {	// Short press
			next_mode();
		} 
		else {
		
		#ifdef TURBO	// Only do turbo check when switch isn't pressed
			if (mode_idx == highest_mode) {
				turbo_ticks++;
				if (turbo_ticks > TURBO_TIMEOUT) {	// Go to the previous mode
					prev_mode();
				}
			}
		#endif

		#ifdef VOLTAGE_MON	// Only do voltage monitoring when the switch isn't pressed
			if (adc_ticks > 0) {
				--adc_ticks;
			}
			if (adc_ticks == 0) {
				if (ADCSRA & (1 << ADIF)) {	// See if conversion is done
					if (ADCH < ((mode_idx == 1) ? ADC_CRIT : ADC_LOW)) {
						++lowbatt_cnt;	// See if voltage is lower than what we were looking for
					} else {
						lowbatt_cnt = 0;
					}
				}
				
				if (lowbatt_cnt >= 4) {	// See if it's been low for a while

					uint8_t i = 0;
					while (i++<3) {
						prev_mode();
						set_output();
						_delay_ms(250);
						next_mode();
						set_output();
						_delay_ms(500);
						}

					prev_mode();
					lowbatt_cnt = 0;	// If we reach 0 here, main loop will go into sleep mode
					adc_ticks = ADC_DELAY;	// Restart the counter to when we step down again
				}
				
				ADCSRA |= (1 << ADSC);	// Make sure conversion is running for next time through
			}
		#endif
		}
		press_duration = 0;
	}
}

int main(void)
{	
	DDRB = 0x00;	// Set all ports to input, and turn pull-up resistors on for the inputs we are using
	PORTB = (1 << SWITCH_PIN);	// | (1 << STAR3_PIN);
	PCMSK = (1 << SWITCH_PIN);	// Set the switch as an interrupt for when we turn pin change interrupts on
	
   	DDRB |= (1 << Pin_6);	DDRB |= (1 << Pin_5);	DDRB |= (1 << Pin_3);	// set pins as output
	
	TCCR0B = 0x01;		// set pre-scaler for timer (1 => 1, 2 => 8, 3 => 64...)

 	#ifdef VOLTAGE_MON
	ADC_on();
	#else
	ADC_off();
	#endif
	ACSR   |=  (1<<7); //AC off
	
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);	// Enable sleep mode set to Power Down that will 
	sleep_until_switch_press();				// be triggered by the sleep_mode() command.
	
	uint8_t last_mode_idx = 0;
	
	while(1) {
		// We will never leave this loop.  The WDT will interrupt to check for switch presses and 
		// will change the mode if needed.  If this loop detects that the mode has changed, run the
		// logic for that mode while continuing to check for a mode change.
		if (mode_idx != last_mode_idx) {	// The WDT changed the mode.

			set_output();

			last_mode_idx = mode_idx;
			
			if (mode_idx == 0) {
				_delay_ms(1); // Need this here, maybe instructions for PWM output not getting executed before shutdown?
				sleep_until_switch_press();	// Go to sleep
			}
		}
	}

    return 0; // Standard Return Code
}