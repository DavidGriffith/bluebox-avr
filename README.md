**Bluebox AVR**
===============

***Bluebox AVR*** implements a bluebox (see 
https://en.wikipedia.org/wiki/Blue_box) on AVR microcontrollers.  This 
project is roughly a translation of the PIC-based bluebox at 
http://projectmf.org/bluebox.html.  Schematics and board layouts will 
soon be available in different repositories.


Description
-----------

Currently only 13 keys are supported.  These are arranged in a 3 x 4 
rectangle with the 13th key appearing at the very top by itself.  That 
one is reserved for playing the 2600hz tone.  The rest are as a standard 
telephone keypad.


Operation
---------

There are currently five tone modes:
1. MF:  These emit MF tones 0 through 9 with KP and ST -- a standard bluebox.
2. DTMF:  Standard DTMF dialing tones
3. Redbox:
	|              |             |               |
	|--------------|-------------|---------------|
	| US Nickel    | US dime     | US quarter    |
	|Canada nickel | Canada dime | Canada quarter|
	| UK 10 pence  | UK 50 pence |      .        |
	|     .        |      .      |      .        |

4. Greenbox (the first two rows have a 2600hz wink.  
   The next two rows have a 900hz + 1500hz wink:
	| | | |
	|------------------|-------------------|-----------------------------|
	| Coin collect     | Coin return       | Ringback                    |
	| Operator attach  | Operator release  | Op release and coin collect |
	| Coin collect     | Coin return       | Ringback                    |
	| Operator attach  | Operator release  | Op release and coin collect |
    
5. 2600hz pulse: Emits 2600hz pulses according to the number as on a 
rotary dial (0 is 10 pulses).  This mode predates MF tones.  This was 
how John Draper (aka Cap'n Crunch) and Joe Engressia Jr. (aka 
Joybubbles) were able to phreak using a whistled 2600hz tone.

Mode is selected by holding down the key corresponding to the 
mode's number while switching the unit on.  A 1700hz tone will play to 
let you know that you've switched modes.  To set the startup mode, hold 
the 2600hz key while turning the unit on.  This will cause a 1700hz tone 
to play.  Then press a key for the mode you want to set as the startup 
mode.  Two tones will then play to let you know that your desired mode 
has been saved to memory.

On startup and when setting startup mode, if you hold the hash key (\#), 
this will toggle between a tone length and interval of 75 milliseconds 
and 120 milliseconds.  Currently this setting is not saved.
