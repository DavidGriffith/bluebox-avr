/*
 * Name:	bluebox.c
 * Author:	David Griffith <dave@661.org>
 * Date:	September 1, 2016
 * License:	?
 * Version:	0
 *
 * Fuse Settings:	L:FF H:DF
 *
 * This program implements a bluebox with PWM synthesis on an AVR
 * ATtiny85 8-pin microcontroller.  A single pin detects 13 buttons
 * through an ADC using a resistor ladder.  There are 12 memory slots
 * of up to 32 tones each.  Defaults are configurable.
 *
 * This is a rough translation / reimplementation of Don Froula's
 * PicBasicPro program for implementing a bluebox on a PIC12F683 8-pin
 * microcontroller.  See http://projectmf.org/ for more information and
 * on VOIP servers modified to accept MF tones and how to make your
 * server do the same.
 *
 * For all you naysayers, this program and the hardware on which it runs
 * are perfectly legal now.  The modern commercial switching offices have
 * long ago made blueboxing and redboxing impossible on the public phone
 * networks.  You can still do it on private networks specifically
 * programmed to allow for this kind of thing.
 *
 * Resistor network detection values assume a network of fourteen 1K ohm
 * resistors in series, from Vdd to Vss, with a tap between each
 * resistor pair.  Taps at Vdd and Vss are not used, to avoid ADC issues
 * when trying to read at the voltage rails.
 *
 */

#include <stdlib.h>
#include <stdint.h>

#include <util/delay.h>		/* for _delay_ms() */
#include <avr/io.h>
#include <avr/interrupt.h>	/* for sei() */
#include <avr/pgmspace.h>

/*
 * Sine samples 7-bit resolution, range 0-126, 256 samples
 *
 * http://www.daycounter.com/Calculators/Sine-Generator-Calculator2.phtml
 *
 */
const unsigned char sine_table[] PROGMEM = {
0x3f, 0x41, 0x42, 0x44, 0x45, 0x47, 0x48, 0x4a,
0x4b, 0x4d, 0x4e, 0x50, 0x51, 0x53, 0x54, 0x56,
0x57, 0x59, 0x5a, 0x5b, 0x5d, 0x5e, 0x5f, 0x61,
0x62, 0x63, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a,
0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73,
0x73, 0x74, 0x75, 0x76, 0x77, 0x77, 0x78, 0x79,
0x79, 0x7a, 0x7a, 0x7b, 0x7b, 0x7c, 0x7c, 0x7c,
0x7d, 0x7d, 0x7d, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e,
0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7d, 0x7d,
0x7d, 0x7c, 0x7c, 0x7c, 0x7b, 0x7b, 0x7a, 0x7a,
0x79, 0x79, 0x78, 0x77, 0x77, 0x76, 0x75, 0x74,
0x73, 0x73, 0x72, 0x71, 0x70, 0x6f, 0x6e, 0x6d,
0x6c, 0x6a, 0x69, 0x68, 0x67, 0x66, 0x65, 0x63,
0x62, 0x61, 0x5f, 0x5e, 0x5d, 0x5b, 0x5a, 0x59,
0x57, 0x56, 0x54, 0x53, 0x51, 0x50, 0x4e, 0x4d,
0x4b, 0x4a, 0x48, 0x47, 0x45, 0x44, 0x42, 0x41,
0x3f, 0x3d, 0x3c, 0x3a, 0x39, 0x37, 0x36, 0x34,
0x33, 0x31, 0x30, 0x2e, 0x2d, 0x2b, 0x2a, 0x28,
0x27, 0x25, 0x24, 0x23, 0x21, 0x20, 0x1f, 0x1d,
0x1c, 0x1b, 0x19, 0x18, 0x17, 0x16, 0x15, 0x14,
0x12, 0x11, 0x10, 0x0f, 0x0e, 0x0d, 0x0c, 0x0b,
0x0b, 0x0a, 0x09, 0x08, 0x07, 0x07, 0x06, 0x05,
0x05, 0x04, 0x04, 0x03, 0x03, 0x02, 0x02, 0x02,
0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01,
0x01, 0x02, 0x02, 0x02, 0x03, 0x03, 0x04, 0x04,
0x05, 0x05, 0x06, 0x07, 0x07, 0x08, 0x09, 0x0a,
0x0b, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11,
0x12, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1b,
0x1c, 0x1d, 0x1f, 0x20, 0x21, 0x23, 0x24, 0x25,
0x27, 0x28, 0x2a, 0x2b, 0x2d, 0x2e, 0x30, 0x31,
0x33, 0x34, 0x36, 0x37, 0x39, 0x3a, 0x3c, 0x3d
};

#define TRUE	1
#define FALSE	0

#define DEBOUNCE_TIME	25

#define MODE_MF		0x01
#define MODE_DTMF	0x02
#define MODE_REDBOX	0x03
#define MODE_GREENBOX	0x04

#define SEIZE_LENGTH	1000
#define SEIZE_PAUSE	1500
#define KP_LENGTH	120


#define SINE_SAMPLES	255UL
#define TICKS_PER_CYCLE	256UL
#define SINE_MIDPOINT	0x7F	// After decoupling, this is 0V of the sine
#define STEP_SHIFT	6
#define SAMPLES_PER_HERTZ_TIMES_256	(SINE_SAMPLES * (TICKS_PER_CYCLE << STEP_SHIFT)) / (F_CPU / 256)
#define OVERFLOW_PER_MILLISEC (F_CPU / TICKS_PER_CYCLE / 1000)

#define cbi(var, mask)	((var) &= (uint8_t)~(1 << mask))
#define sbi(var, mask)	((var) |= (uint8_t)(1 << mask))

#define TIMER0_PRESCALE_1	(1<<CS00)
#define TIMER0_PRESCALE_8	(1<<CS01)
#define TIMER0_PRESCALE_64	((1<<CS01)|(1<<CS00))
#define TIMER0_PRESCALE_256	(1<<CS02)
#define TIMER0_PRESCALE_1024	((1<<CS02)|(1<<CS00))

#define TIMER0_ON(x) \
do { \
    TCCR0A = ((1<<COM0A1)|(1<<WGM01)|(1<<WGM00)); \
    TCCR0B = (x);\
} while (0)
#define TIMER0_OFF()	TCCR0A &= ~((1<<CS02)|(1<<CS01)|(1<<CS00))
#define TONES_ON()	TIMER0_ON(TIMER0_PRESCALE_1)
#define TONES_OFF()	TIMER0_OFF()

#define TONE_LENGTH_FAST	75
#define TONE_LENGTH_SLOW	120

#define KEY_1		1
#define KEY_2		2
#define KEY_3		3
#define KEY_4		4
#define KEY_5		5
#define KEY_6		6
#define KEY_7		7
#define KEY_8		8
#define KEY_9		9
#define KEY_STAR	10
#define KEY_0		11
#define KEY_HASH	12
#define KEY_SEIZE	13

typedef uint8_t bool;

uint8_t tone_mode = MODE_MF;
uint8_t tone_length = TONE_LENGTH_FAST;
bool  playback_mode = FALSE;
bool  tones_on = FALSE;

uint16_t tone_a_step, tone_b_step;
uint16_t tone_a_place, tone_b_place;

void  init_ports(void);
void  init_settings(void);
void  init_adc(void);
uint8_t getkey(void);
void  process(uint8_t);
void  play(uint32_t, uint32_t, uint32_t);
void  sleep_ms(uint16_t ms);
void  tick(void);

static uint8_t millisec_counter = OVERFLOW_PER_MILLISEC;
static volatile uint8_t millisec_flag;

int main(void)
{
	uint8_t key;

//	init_settings();
	init_ports();
	init_adc();

	millisec_flag = 0;


	// Start TIMER0
	TCCR0A = ((1<<COM0A1)|(1<<WGM01)|(1<<WGM00));
	TCCR0B = (TIMER0_PRESCALE_1);

	key = getkey();

	switch (key) {
	case KEY_1:	tone_mode = MODE_MF; break;
	case KEY_2:	tone_mode = MODE_DTMF; break;
	case KEY_3:	tone_mode = MODE_REDBOX; break;
	case KEY_4:	tone_mode = MODE_GREENBOX; break;
	// Toggle power-up tone mode and store in EE if '*' held at power-up
	case KEY_STAR:	break;
	// Toggle power-up tone length and store in EE if '#' held at power-up
	case KEY_HASH:	break;
	}

	if (key > 0) play(1000, 1700, 1700);
	while (key == getkey());	// Wait for key to be released

	// Normal operation happens here
	while (1) {
		key = getkey();
		process(key);
		while (key == getkey());	// Wait for key to be released
	}
	return 0;

} /* void main() */



/*
 * void process(uint8_t key)
 *
 * Process regular keystroke
 *
 */
void process(uint8_t key)
{
	if (key == 0) return;

	// If playing memories, set the correct EE bank to play back
	if (playback_mode == FALSE) {
		// stuff to set up tone_length and so on.
	}

	if (tone_mode == MODE_MF) {
		switch (key) {
		case KEY_1: play(tone_length, 700, 900); break;
		case KEY_2: play(tone_length, 700, 1100); break;
		case KEY_3: play(tone_length, 900, 1100); break;
		case KEY_4: play(tone_length, 700, 1300); break;
		case KEY_5: play(tone_length, 900, 1300); break;
		case KEY_6: play(tone_length, 1100, 1300); break;
		case KEY_7: play(tone_length, 700, 1500); break;
		case KEY_8: play(tone_length, 900, 1500); break;
		case KEY_9: play(tone_length, 1100, 1500); break;
		case KEY_STAR: play(KP_LENGTH, 1100, 1700); break;
		case KEY_0: play(tone_length, 1300, 1500); break;
		case KEY_HASH: play(tone_length, 1500, 1700); break;
		case KEY_SEIZE: play(SEIZE_LENGTH, 2600, 2600);
			 if (playback_mode == TRUE)
				sleep_ms(SEIZE_PAUSE);
			break;
		}
	} else if (tone_mode == MODE_DTMF) {
		switch (key) {
		case KEY_1: play(tone_length, 697, 1209); break;
		case KEY_2: play(tone_length, 697, 1336); break;
		case KEY_3: play(tone_length, 697, 1477); break;
		case KEY_4: play(tone_length, 770, 1209); break;
		case KEY_5: play(tone_length, 770, 1336); break;
		case KEY_6: play(tone_length, 770, 1477); break;
		case KEY_7: play(tone_length, 852, 1209); break;
		case KEY_8: play(tone_length, 852, 1336); break;
		case KEY_9: play(tone_length, 852, 1477); break;
		case KEY_STAR: play(tone_length, 941, 1209); break;
		case KEY_0: play(tone_length, 941, 1336); break;
		case KEY_HASH: play(tone_length, 941, 1477); break;
		case KEY_SEIZE: play(SEIZE_LENGTH, 2600, 2600);
			 if (playback_mode == TRUE)
				sleep_ms(SEIZE_PAUSE);
			break;
		}
	} else if (tone_mode == MODE_REDBOX) {
		switch (key) {
		case KEY_1: play(66, 1700, 2200);	// Nickel
			break;
		case KEY_2: play(66, 1700, 2200);	// Dime
			sleep_ms(66);
			play(66, 1700, 2200);
			break;
		case KEY_3: play(33, 1700, 2200);	// Quarter
			sleep_ms(33);
			play(33, 1700, 2200);
			sleep_ms(33);
			play(33, 1700, 2200);
			sleep_ms(33);
			play(33, 1700, 2200);
			sleep_ms(33);
			play(33, 1700, 2200);
			break;
		case KEY_4: play(200, 1000, 1000);	// 10 pence
			break;
		case KEY_5: play(350, 1000, 1000);  	// 50 pence
			break;
		case KEY_SEIZE: play(SEIZE_LENGTH, 2600, 2600);
			 if (playback_mode == TRUE)
				sleep_ms(SEIZE_PAUSE);
			break;
		}
	} else if (tone_mode == MODE_GREENBOX) {
		switch(key) {
		// With 2600 wink
		case KEY_1: play(90, 2600, 2600);	// coin collect
			sleep_ms(60);
			play(900, 700, 1100);
			break;
		case KEY_2: play(90, 2600, 2600);	// coin return
			sleep_ms(60);
			play(900, 1100, 1700);
			break;
		case KEY_3: play(90, 2600, 2600);	// ringback
			sleep_ms(6);
			play(900, 700, 1700);
			break;
		// With MF "8" (900 Hz + 1500 Hz) wink
		case KEY_4: play(90, 900, 1500);	// coin collect
			sleep_ms(60);
			play(900, 700, 1100);
			break;
		case KEY_5: play(90, 900, 1500);	// coin return
			sleep_ms(60);
			play(900, 1100, 1700);
			break;
		case KEY_6: play(90, 900, 1500);	// ringback
			sleep_ms(6);
			play(900, 700, 1700);
			break;
		case KEY_SEIZE: play(SEIZE_LENGTH, 2600, 2600);
			 if (playback_mode == TRUE)
				sleep_ms(SEIZE_PAUSE);
			break;
		}
	}
}


/*
 * void play(uint32_t duration, uint32_t freq_a, uint32_t freq_b)
 *
 * Plays a pair of tones (in Hz) for the duration (in ms) specified.
 *
 */
void play(uint32_t duration, uint32_t freq_a, uint32_t freq_b)
{
	uint32_t tmp_a = SAMPLES_PER_HERTZ_TIMES_256;
	uint32_t tmp_b = SAMPLES_PER_HERTZ_TIMES_256;

	tmp_a *= freq_a;
	tone_a_step = tmp_a / 256;
	tmp_b *= freq_b;
	tone_b_step = tmp_b / 256;

	tone_a_place = 0;
	tone_b_place = 0;
	tones_on = TRUE;
	sleep_ms(duration);
	tones_on = FALSE;
}


/*
 * void sleep_ms(uint16_t milliseconds)
 *
 * For some strange reason, _delay_ms() and _delay_us() run too slow by
 * a factor of around four.  I don't know if this is particular to the
 * ATtiny line or if there's a problem in the Linux AVR development
 * tools.  Anyhow, this function simply holds up execution by the
 * supplied number of milliseconds.  The tick() function can be
 * convenient for monitoring buttons and doing debouncing.  More on that
 * later.
 *
 */
void sleep_ms(uint16_t milliseconds)
{
	while( milliseconds > 0 ) {
		if( millisec_flag ) {
			millisec_flag = 0;
			milliseconds--;
			tick();
		}
	}
}

void tick(void) {}


ISR(TIM0_OVF_vect)
{
	if (tones_on) {
	OCR0A = pgm_read_byte(&(sine_table[(tone_a_place >> STEP_SHIFT)])) +
		pgm_read_byte(&(sine_table[(tone_b_place >> STEP_SHIFT)]));
	tone_a_place += tone_a_step;
	tone_b_place += tone_b_step;
	if(tone_a_place >= (SINE_SAMPLES << STEP_SHIFT))
		tone_a_place -= (SINE_SAMPLES << STEP_SHIFT);
	if(tone_b_place >= (SINE_SAMPLES << STEP_SHIFT))
		tone_b_place -= (SINE_SAMPLES << STEP_SHIFT);
	} else OCR0A = SINE_MIDPOINT;

	// count milliseconds
	millisec_counter--;
	if(millisec_counter == 0) {
		millisec_counter = OVERFLOW_PER_MILLISEC;
		millisec_flag = 1;
	}
}


/*
 * uint8_t getkey(void)
 *
 * Returns the number of key pressed (1-13) or 0 if no key was pressed
 *
 * The resistor ladder feeds a voltage ranging from 0 VDC up to around
 * 4.64 VDC into the ADC pin.  The AVR then samples it and gives an
 * 8-bit value proportional to the voltage as compared to Vdd.  Then we
 * check to see what range that value falls into and thus we know which
 * button was pressed.
 *
 * Any ADC value less than 9 is essentially 0 VDC because of the
 * pull-down resistor, with some margin for noise.  This means that no
 * key has been pressed.
 *
 */
uint8_t getkey(void)
{
	uint8_t voltage;
	while (1) {
		ADCSRA |= (1 << ADSC);		// start ADC measurement
		while (ADCSRA & (1 << ADSC) );	// wait till conversion complete
		sleep_ms(DEBOUNCE_TIME/3);	// delay for debounce
		voltage = ADCH;
		ADCSRA |= (1 << ADSC);		// start ADC measurement
		while (ADCSRA & (1 << ADSC) );	// wait till conversion complete
		if (voltage != ADCH) continue;	// bouncy result, try again
		if (voltage <  9) return 0;	// no key has been pressed

		// If we made it this far, then we've got something valid
		// These values calculated with Vdd = 5 volts DC

		// 4.64 volts.  ADC value = 246
		if (ADCH > 233 && ADCH <= 256) return KEY_1;
		// 4.29 volts.  ADC value = 219
		if (ADCH > 211 && ADCH <= 232) return KEY_2;
		// 3.93 volts.  ADC value = 201
		if (ADCH > 192 && ADCH <= 210) return KEY_3;
		// 3.57 volts.  ADC value = 183
		if (ADCH > 174 && ADCH <= 191) return KEY_4;
		// 3.21 volts.  ADC value = 165
		if (ADCH > 155 && ADCH <= 173) return KEY_5;
		// 2.86 volts.  ADC value = 146
		if (ADCH > 137 && ADCH <= 154) return KEY_6;
		// 2.50 volts.  ADC value = 128
		if (ADCH > 119 && ADCH <= 136) return KEY_7;
		// 2.14 volts.  ADC value = 110
		if (ADCH > 101 && ADCH <= 118) return KEY_8;
		// 1.79 volts.  ADC value = 91
		if (ADCH > 82  && ADCH <= 100) return KEY_9;
		// 1.42 volts.  ADC value = 73
		if (ADCH > 64  && ADCH <=  81) return KEY_STAR;
		// 1.07 volts.  ADC value = 55
		if (ADCH > 46  && ADCH <=  63) return KEY_0;
		// 0.71 volts.  ADC value = 37
		if (ADCH > 27  && ADCH <=  45) return KEY_HASH;
		// 0.357 volts.  ADC value = 18
		if (ADCH > 9   && ADCH <=  26) return KEY_SEIZE;
		// We shouldn't get past here,
		// but if we do, treat it like no key detected.
		break;
	}
	return 0;
}  /* uint8_t getkey() */


void init_ports(void)
{
	cli();
	// PB0 is output, PB2 is input
	// PB3 and PB4 are for the crystal
	DDRB  = 0b11100011;
	TIMSK |= (1<<TOIE0);
	sei();
}


/*
 * void init_adc(void)
 *
 * ADC prescaler needs to be set so that the ADC input frequency is
 * between 50 -- 200kHz
 *
 * For more information, see table 17.5 "ADC Prescaler Selections" in
 * chapter 17.13.2 "ADCSRA – ADC Control and Status Register A"
 * (pages 140 and 141 on the complete ATtiny25/45/85 datasheet,
 * Rev. 2586M–AVR–07/10)
 *
 */
void init_adc()
{
	// 8-bit resolution
	// set ADLAR to 1 to enable the left-shift result
	// (only bits ADC9..ADC2 are available), then
	// only reading ADCH is sufficient for 8-bit results (256 values)
	ADMUX =
		(1 << ADLAR) |	// left shift result
		(0 << REFS1) |	// set ref voltage to VCC, bit 1
		(0 << REFS0) |	// set ref voltage to VCC, bit 0
		(0 << MUX3)  |	// use ADC1 for input (PB2), MUX bit 3
		(0 << MUX2)  |  // use ADC1 for input (PB2), MUX bit 2
		(0 << MUX1)  |  // use ADC1 for input (PB2), MUX bit 1
		(1 << MUX0);	// use ADC1 for input (PB2), MUX bit 0

	// using a 20MHz crystal.  setting prescaler to 128 gives me a
	// frequency of 156.250 kHz
	ADCSRA =
		(1 << ADEN)  |	// enable ADC
		(1 << ADPS2) |	// set prescaler to 128, bit 2
		(1 << ADPS1) |	// set prescaler to 128, bit 1
		(1 << ADPS0);	// set prescaler to 128, bit 0
	return;
} /* void init_adc() */
