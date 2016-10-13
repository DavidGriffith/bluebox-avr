/*
 * Name:	bluebox.c
 * Author:	David Griffith <dave@661.org>
 * Date:	September 1, 2016
 * License:	?
 * Version:	0
 *
 * Fuse Settings:	L:FF H:DF
 *
 * This program implements a bluebox, redbox, and greenbox with PWM
 * synthesis on an AVR ATtiny85 8-pin microcontroller.  A single pin
 * detects 13 buttons through an ADC using a resistor ladder.  There are
 * 12 memory slots of up to 32 tones each.  Defaults are configurable.
 *
 * This is a rough translation / reimplementation / expansion of Don Froula's
 * PicBasicPro program for implementing a bluebox on a PIC12F683 8-pin
 * microcontroller.  See http://projectmf.org/ for more information on
 * this, for information on VOIP servers modified to accept MF tones
 * and how to make your server do the same.
 *
 * For all you naysayers, this program and the hardware on which it runs
 * are perfectly legal now.  The modern commercial switching offices have
 * long ago made blueboxing and redboxing impossible on the public phone
 * networks.  You can still do it on private networks specifically
 * programmed to allow for this kind of thing.
 *
 * Resistor network detection values assume a network of fourteen (for
 * 13-key keypad) or seventeen (for 16-key keypad) 1K ohm
 * resistors in series, from Vdd to Vss, with a tap between each
 * resistor pair.  Taps at Vdd and Vss are not used, to avoid ADC issues
 * when trying to read at the voltage rails.
 *
 * Many thanks go to the denizens of #AVR on irc.freenode.net for
 * answering my questions on sine wave generation.
 *
 */

/*
 * One and only one of these must be uncommented.
 * The REV ones are for buttons mounted on the solder side.
 *
 */
#define KEYPAD_13
//#define KEYPAD_13_REV
//#define KEYPAD_16
//#define KEYPAD_13_REV

#include <stdlib.h>
#include <stdint.h>

#include <util/delay.h>		/* for _delay_ms() */
#include <avr/io.h>
#include <avr/interrupt.h>	/* for sei() */
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

/*
 * Sine samples 7-bit resolution, range 0-126, 256 samples
 *
 * http://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
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

/*
 * The tone mode is stored as the first byte of a memory chunk.
 * When eeprom is initialized as the AVR is programmed, the
 * default state of each memory address is 0xFF.  Therefore, when
 * we find a chunk beginning with 0xFF, we conclude that the
 * memory location is empty.
 *
 */
#define MODE_EMPTY	0xFF
#define MODE_MF		0x00
#define MODE_DTMF	0x01
#define MODE_REDBOX	0x02
#define MODE_GREENBOX	0x03
#define MODE_PULSE	0x04

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

#define TONE_LENGTH_FAST	75
#define TONE_LENGTH_SLOW	120

#define KEY_NOTHING	0
#ifdef KEYPAD_13
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
#elif KEYPAD_13_REV
#define KEY_1		3
#define KEY_2		2
#define KEY_3		1
#define KEY_4		6
#define KEY_5		5
#define KEY_6		4
#define KEY_7		9
#define KEY_8		8
#define KEY_9		7
#define KEY_STAR	12
#define KEY_0		11
#define KEY_HASH	10
#define KEY_SEIZE	13
#elif KEYPAD_16
#define KEY_1		1
#define KEY_2		2
#define KEY_3		3
#define KEY_A		4
#define KEY_4		5
#define KEY_5		6
#define KEY_6		7
#define KEY_B		8
#define KEY_7		9
#define KEY_8		10
#define KEY_9		11
#define KEY_C		12
#define KEY_STAR	13
#define KEY_0		14
#define KEY_HASH	15
#define KEY_D		16
#elif KEYPAD_16_REV
#define KEY_1		4
#define KEY_2		3
#define KEY_3		2
#define KEY_A		1
#define KEY_4		8
#define KEY_5		7
#define KEY_6		6
#define KEY_B		5
#define KEY_7		12
#define KEY_8		11
#define KEY_9		10
#define KEY_C		9
#define KEY_STAR	16
#define KEY_0		15
#define KEY_HASH	14
#define KEY_D		13
#else
#error One and only one of the following must be defined: KEYPAD_13 KEYPAD_13_REV KEYPAD_16  KEYPAD_16_REV
#endif

/* Number of milliseconds to make for a long press */
#define LONGPRESS_TIME	2000

/* two bytes, then 12 chunks of 41 bytes each */
#define EEPROM_CHUNK_SIZE			0x29
#define EEPROM_STARTUP_TONE_MODE		0x01
#define EEPROM_STARTUP_TONE_LENGTH		0x02
#define EEPROM_MEM1				0x03
#define EEPROM_MEM2				EPROM_MEM1 + EEPROM_CHUNK_SIZE
#define EEPROM_MEM3				EPROM_MEM2 + EEPROM_CHUNK_SIZE
#define EEPROM_MEM4				EPROM_MEM3 + EEPROM_CHUNK_SIZE
#define EEPROM_MEM5				EPROM_MEM4 + EEPROM_CHUNK_SIZE
#define EEPROM_MEM6				EPROM_MEM5 + EEPROM_CHUNK_SIZE
#define EEPROM_MEM7				EPROM_MEM6 + EEPROM_CHUNK_SIZE
#define EEPROM_MEM8				EPROM_MEM7 + EEPROM_CHUNK_SIZE
#define EEPROM_MEM9				EPROM_MEM8 + EEPROM_CHUNK_SIZE
#define EEPROM_MEM10				EPROM_MEM9 + EEPROM_CHUNK_SIZE
#define EEPROM_MEM11				EPROM_MEM10 + EEPROM_CHUNK_SIZE
#define EEPROM_MEM12				EPROM_MEM11 + EEPROM_CHUNK_SIZE

typedef uint8_t bool;

uint8_t tone_mode;
uint8_t tone_length;
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
void  pulse(uint8_t);

void  sleep_ms(uint16_t ms);
void  tick(void);
static uint8_t millisec_counter = OVERFLOW_PER_MILLISEC;
static volatile uint8_t millisec_flag = FALSE;

void longpress_start(void);
void longpress_stop(void);
static uint16_t	longpress_counter;
static uint8_t	longpress_on = FALSE;
static volatile uint8_t longpress_flag = FALSE;

uint8_t ee_data[] EEMEM = {0,0,75};

int main(void)
{
	uint8_t key;
	bool	startup_set = FALSE;

	init_ports();
	init_adc();

	// Start TIMER0
	// The timer is counting from 0 to 255 -- 256 values.
	// The prescaler is 1.  Therefore our PWM frequency is F_CPU / 256.
	TIMER0_ON(TIMER0_PRESCALE_1);

	// Read setup bytes
	tone_mode = eeprom_read_byte(( uint8_t *)EEPROM_STARTUP_TONE_MODE);

	if (tone_mode == 0xff) {
		play(75,1000,1000);
		sleep_ms(66);
		play(75,1000,1000);
		sleep_ms(66);
		play(75,1000,1000);
		sleep_ms(66);
		play(75,1000,1000);
		sleep_ms(66);
		play(75,1000,1000);
		sleep_ms(66);
		play(75,1000,1000);
		sleep_ms(1000);
	}

	if (tone_mode < MODE_MF || tone_mode > MODE_PULSE) {
		tone_mode = MODE_MF;		// Set MODE_MF if bogus
		play(75,880,880);
		sleep_ms(66);
		play(75,880,880);
		sleep_ms(66);
		play(75,880,880);
		sleep_ms(66);
	}

	tone_length = eeprom_read_byte(( uint8_t *)EEPROM_STARTUP_TONE_LENGTH);
	if (tone_length < TONE_LENGTH_FAST || tone_length > TONE_LENGTH_SLOW)
		tone_length = TONE_LENGTH_FAST;

	key = getkey();		// What key is held on startup?

	if (key == KEY_SEIZE) {	// We're setting a default mode
		startup_set = TRUE;
		play(1000, 1700, 1700);
		while (key == getkey());	// Wait for release
		do {				// Get the next keystroke
			key = getkey();
		} while (key == KEY_NOTHING);
	}

	switch (key) {
	case KEY_1:	tone_mode = MODE_MF; break;
	case KEY_2:	tone_mode = MODE_DTMF; break;
	case KEY_3:	tone_mode = MODE_REDBOX; break;
	case KEY_4:	tone_mode = MODE_GREENBOX; break;
	case KEY_5:	tone_mode = MODE_PULSE; break;
	case KEY_HASH:	if (tone_length == TONE_LENGTH_FAST)
				tone_length = TONE_LENGTH_SLOW;
			else
				tone_length = TONE_LENGTH_FAST;
			break;
	default:	play(1000, 440, 440);
			break;
	}

	if (startup_set) {
		play(75, 1700, 1700);
		eeprom_update_byte(( uint8_t *)EEPROM_STARTUP_TONE_MODE, tone_mode);
		eeprom_update_byte(( uint8_t *)EEPROM_STARTUP_TONE_LENGTH, tone_length);
		eeprom_busy_wait();
		play(1000, 1500, 1500);
	} else {
		if (key > KEY_NOTHING) play(1000, 1700, 1700);
	}

	while (key == getkey());	// Wait for key to be released

	// Normal operation happens here
	while (1) {

		do { key = getkey(); }
		while (key == KEY_NOTHING);

		process(key);

		// Wait for release or long-press
		longpress_start();
		while (key == getkey() && key != KEY_NOTHING) {
			if (longpress_flag) {
				play(75, 1700, 1700);

				// do long-press stuff here

				play(1000, 1500, 1500);
				while (key == getkey());
				break;
			}
		}
		longpress_stop();
	}
	return 0;
} /* void main() */


#if defined(KEYPAD_16) || defined(KEYPAD_16_REV)
#error 16-keys not yet implemented
/*
 * void process(uint8_t key)
 *
 * Process regular keystroke for 16-key keypad
 *
 */
void process(uint8_t key)
{
	// nothing here yet
}

uint8_t getkey(void)
{
	// nothing here yet
}

#else	// We're using a 13-key keypad

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

	// The 2600 key always plays 2600, so catch it here.
	if (key == KEY_SEIZE) {
		play(SEIZE_LENGTH, 2600, 2600);
		if (playback_mode == TRUE)
			sleep_ms(SEIZE_PAUSE);
		return;
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
		case KEY_4: play(60, 2200, 2200);	// Canada nickel
			break;
		case KEY_5: play(60, 2200, 2200);	// Canada dime
			sleep_ms(60);
			play(60, 2200, 2200);
			sleep_ms(60);
			break;
		case KEY_6: play(33, 2200, 2200);	// Canada quarter
			sleep_ms(33);
			play(33, 2200, 2200);
			sleep_ms(33);
			play(33, 2200, 2200);
			sleep_ms(33);
			play(33, 2200, 2200);
			sleep_ms(33);
			play(33, 2200, 2200);
			sleep_ms(33);
			break;
		case KEY_7: play(200, 1000, 1000);	// UK 10 pence
			break;
		case KEY_8: play(350, 1000, 1000);  	// UK 50 pence
			break;
		}
	} else if (tone_mode == MODE_GREENBOX) {
		switch(key) {
		// Using 2600 wink
		case KEY_1: play(90, 2600, 2600);	// Coin collect
			sleep_ms(60);
			play(900, 700, 1100);
			break;
		case KEY_2: play(90, 2600, 2600);	// Coin return
			sleep_ms(60);
			play(900, 1100, 1700);
			break;
		case KEY_3: play(90, 2600, 2600);	// Ringback
			sleep_ms(60);
			play(900, 700, 1700);
			break;
		case KEY_4: play(90, 2600, 2600);	// Operator attached
			sleep_ms(60);
			play(700, 1300, 1500);
			break;
		case KEY_5: play(90, 2600, 2600);	// Operator released
			sleep_ms(60);
			play(700, 900, 1500);
			break;
		case KEY_6: play(90, 2600, 2600);	// Operator release
			sleep_ms(60);			// and coin collect
			play(700, 1500, 1700);
			break;
		// With MF "8" (900 Hz + 1500 Hz) wink
		case KEY_7: play(90, 900, 1500);	// Coin collect
			sleep_ms(60);
			play(900, 700, 1100);
			break;
		case KEY_8: play(90, 900, 1500);	// Coin return
			sleep_ms(60);
			play(900, 1100, 1700);
			break;
		case KEY_9: play(90, 900, 1500);	// Ringback
			sleep_ms(60);
			play(900, 700, 1700);
			break;
		case KEY_STAR: play(90, 900, 1500);	// Operator attached
			sleep_ms(60);
			play(700, 1300, 1500);
			break;
		case KEY_0: play(90, 900, 1500);	// Operator released
			sleep_ms(60);
			play(700, 900, 1500);
			break;
		case KEY_HASH: play(90, 900, 1500);	// Operator release
			sleep_ms(60);			// and coin collect
			play(700, 1500, 1700);
			break;
		}
	} else if (tone_mode == MODE_PULSE) {
		switch (key) {
		case KEY_1: pulse(1); break;
		case KEY_2: pulse(2); break;
		case KEY_3: pulse(3); break;
		case KEY_4: pulse(4); break;
		case KEY_5: pulse(5); break;
		case KEY_6: pulse(6); break;
		case KEY_7: pulse(7); break;
		case KEY_8: pulse(8); break;
		case KEY_9: pulse(9); break;
		case KEY_0: pulse(10); break;
		}
	}
} /* void process(uint8_t key) */


void longpress_start(void)
{
	longpress_counter = LONGPRESS_TIME;
	longpress_on = TRUE;
}


void longpress_stop(void)
{
	longpress_on = FALSE;
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
 * Any ADC value less than 13 (used to be 9) is essentially 0 VDC
 * because of the pull-down resistor, with some margin for noise.  This
 * means that no key has been pressed.
 *
 * Further reading:
 *    https://learn.sparkfun.com/tutorials/voltage-dividers
 *    http://www.marcelpost.com/wiki/index.php/ATtiny85_ADC
 *
 */
uint8_t getkey(void)
{
	uint8_t voltage;
	while (1) {
		ADCSRA |= (1 << ADSC);		// start ADC measurement
		while (ADCSRA & (1 << ADSC) );	// wait till conversion complete
		sleep_ms(DEBOUNCE_TIME);	// delay for debounce
		voltage = ADCH;
		ADCSRA |= (1 << ADSC);		// start ADC measurement
		while (ADCSRA & (1 << ADSC) );	// wait till conversion complete
		if (voltage != ADCH) continue;	// bouncy result, try again
		if (voltage <  13) return 0;	// no key has been pressed

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
		if (ADCH > 16   && ADCH <=  26) return KEY_SEIZE;
		// We shouldn't get past here,
		// but if we do, treat it like no key detected.
		break;
	}
	return 0;
}  /* uint8_t getkey() */
#endif	// #ifdef KEYPAD_16, #else


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
 * http://www.atmel.com/images/atmel-2586-avr-8-bit-microcontroller-attiny25-attiny45-attiny85_datasheet.pdf
 *
 * Further reading:
 *    http://www.marcelpost.com/wiki/index.php/ATtiny85_ADC
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


/*
 * void play(uint32_t duration, uint32_t freq_a, uint32_t freq_b)
 *
 * Plays a pair of tones (in Hz) for the duration (in ms) specified.
 *
 * There are two ways to play a single tone:
 *   1) make freq_a and freq_b the same
 *   2) make freq_b zero
 *
 */
void play(uint32_t duration, uint32_t freq_a, uint32_t freq_b)
{
	uint32_t tmp_a = SAMPLES_PER_HERTZ_TIMES_256;
	uint32_t tmp_b = SAMPLES_PER_HERTZ_TIMES_256;

	tmp_a *= freq_a;
	tone_a_step = tmp_a / 256;
	if (freq_b == 0)
		tmp_b *= freq_a;
	else
		tmp_b *= freq_b;
	tone_b_step = tmp_b / 256;

	tone_a_place = 0;
	tone_b_place = 0;
	tones_on = TRUE;
	sleep_ms(duration);
	tones_on = FALSE;
}


/*
 * void pulse(uint8_t count)
 *
 * Send a series of 2600hz pulses with the same timing as a rotary dialer
 * This pre-dates the US R1/MF signalling system.
 * This was how John Draper (aka Cap'n Crunch) and Joe Engressia Jr.
 * (aka Joybubbles) were able to phreak using a whistled 2600hz tone.
 *
 */
void pulse(uint8_t count)
{
	uint8_t	i;

	for (i = 0; i < count; i++) {
		play(66, 2600, 2600);
		sleep_ms(34);
	}
	sleep_ms(500);
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
 * Further reading:
 *    http://repos.borg.ch/projects/avr_leds/trunk/ws2811/avr/big-led-string.c
 *
 */
void sleep_ms(uint16_t milliseconds)
{
	while( milliseconds > 0 ) {
		if( millisec_flag ) {
			millisec_flag = FALSE;
			milliseconds--;
			tick();
		}
	}
}

void tick(void) {}


/*
 * ISR(TIM0_OVF_vect)
 *
 * Here we set up a timer to present sine data to an oscillator.
 * This produces a pulse-width-modulated wave that creates an
 * approximation of a sine wave.  This is smoothed out with a low-pass
 * filter after the signal exits the microcontroller.
 *
 * This timer also provides the timing for the sleep_ms() function.
 *
 * Further reading:
 *    https://en.wikipedia.org/wiki/Pulse-width_modulation
 *    https://learn.sparkfun.com/tutorials/pulse-width-modulation
 *
 */
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
		millisec_flag = TRUE;

		// This is a secondary millisecond counter that is turned
		// on only when we're waiting for a key to be pressed
		// and held.  If it times out, then we set a flag to let
		// the main loop know that a long press has occurred.
		if (longpress_on) {
			longpress_counter--;
			longpress_flag = 0;
			if (longpress_counter == 0) {
				longpress_counter = LONGPRESS_TIME;
				longpress_flag = 1;
			}
		}
	}
}
