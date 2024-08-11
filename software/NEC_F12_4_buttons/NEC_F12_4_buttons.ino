// tinyIRremote for ATtiny13A - Multi Protocol, 4 Buttons
// 
// IR remote control using an ATtiny 13A. Timer0 generates a carrier
// frequency with a duty cycle of 25% on the output pin to the
// IR LED. The signal is modulated by toggling the pin to input/output.
//
// The code utilizes the sleep mode power down function. The device will
// work several months on a CR2032 battery.
//
//                        +-\/-+
// KEY5 --- A0 (D5) PB5  1|    |8  Vcc
// KEY3 --- A3 (D3) PB3  2|    |7  PB2 (D2) A1 --- KEY2
// KEY4 --- A2 (D4) PB4  3|    |6  PB1 (D1) ------ IR LED
//                  GND  4|    |5  PB0 (D0) ------ KEY1
//                        +----+    
//
// Controller: ATtiny13
// Core:       MicroCore (https://github.com/MCUdude/MicroCore)
// Clockspeed: 1.2 MHz internal
// BOD:        BOD disabled (energy saving)
// Timing:     Micros disabled (Timer0 is in use)
//
// Note: The internal oscillator may need to be calibrated for the device
//       to function properly.
//
// 2019 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/
//
// Based on the work of:
// - San Bergmans (https://www.sbprojects.net/knowledge/ir/index.php),
// - Christoph Niessen (http://chris.cnie.de/avr/tcm231421.html),
// - David Johnson-Davies (http://www.technoblogy.com/show?UVE).


// oscillator calibration value (uncomment and set if necessary)
//#define OSCCAL_VAL  0x6C

// libraries
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// assign IR commands to the keys
#define KEY1  NEC_sendCode(0x1308,0x87)     // Acer projector Power: addr 0x1308, cmd 0x87 (device 8, subdevice 19, function 135)
#define KEY2  F12_sendCode(7,1,64)     // Elitescreens Vmax up (device, subdevice, function as used in IrScrutinizer)
#define KEY3  F12_sendCode(7,1,128)     // Elitescreens Vmax stop (device, subdevice, function as used in IrScrutinizer)
#define KEY4  F12_sendCode(7,1,32)     // Elitescreens Vmax down (device, subdevice, function as used in IrScrutinizer)
#define KEY5  _delay_ms(10)               // nothing

// macros to switch on/off IR LED
#define IRon()    DDRB |= 0b00000010      // PB1 as output = IR at OC0B (38 kHz)
#define IRoff()   DDRB &= 0b11111101      // PB1 as input  = LED off

// button mask
#define BT_MASK   0b00011101              // port mask for button pins

// -----------------------------------------------------------------------------------
// NEC Protocol Implementation
// -----------------------------------------------------------------------------------
//
// The NEC protocol uses pulse distance modulation.
//
//       +---------+     +-+ +-+   +-+   +-+ +-    ON
//       |         |     | | | |   | |   | | |          bit0:  562.5us
//       |   9ms   |4.5ms| |0| | 1 | | 1 | |0| ...
//       |         |     | | | |   | |   | | |          bit1: 1687.5us
// ------+         +-----+ +-+ +---+ +---+ +-+     OFF
//
// IR telegram starts with a 9ms leading burst followed by a 4.5ms pause.
// Afterwards 4 data bytes are transmitted, least significant bit first.
// A "0" bit is a 562.5us burst followed by a 562.5us pause, a "1" bit is
// a 562.5us burst followed by a 1687.5us pause. A final 562.5us burst
// signifies the end of the transmission. The four data bytes are in order:
// - the 8-bit address for the receiving device,
// - the 8-bit logical inverse of the address,
// - the 8-bit command and
// - the 8-bit logical inverse of the command.
// The Extended NEC protocol uses 16-bit addresses. Instead of sending an
// 8-bit address and its logically inverse, first the low byte and then the
// high byte of the address is transmitted.
//
// If the key on the remote controller is kept depressed, a repeat code
// will be issued consisting of a 9ms leading burst, a 2.25ms pause and
// a 562.5us burst to mark the end. The repeat code will continue to be
// sent out at 108ms intervals, until the key is finally released.

// define values for 38kHz PWM frequency and 25% duty cycle
#define NEC_TOP   31          // 1200kHz / 38kHz - 1 = 31
#define NEC_DUTY  7           // 1200kHz / 38kHz / 4 - 1 = 7

// macros to modulate the signals according to NEC protocol with compensated timings
#define NEC_startPulse()    {IRon(); _delay_us(9000); IRoff(); _delay_us(4500);}
#define NEC_repeatPulse()   {IRon(); _delay_us(9000); IRoff(); _delay_us(2250);}
#define NEC_normalPulse()   {IRon(); _delay_us( 562); IRoff(); _delay_us( 557);}
#define NEC_bit1Pause()     _delay_us(1120) // 1687.5us - 562.5us = 1125us
#define NEC_repeatCode()    {_delay_ms(40); NEC_repeatPulse(); NEC_normalPulse(); _delay_ms(56);}

// send a single byte via IR
void NEC_sendByte(uint8_t value) {
  for (uint8_t i=8; i; i--, value>>=1) {  // send 8 bits, LSB first
    NEC_normalPulse();                    // 562us burst, 562us pause
    if (value & 1) NEC_bit1Pause();       // extend pause if bit is 1
  }
}

// send complete telegram (start frame + address + command) via IR
void NEC_sendCode(uint16_t addr, uint8_t cmd) {
  // prepare carrier wave
  OCR0A  = NEC_TOP;           // set PWM frequency
  OCR0B  = NEC_DUTY;          // set duty cycle

  // send telegram
  NEC_startPulse();           // 9ms burst + 4.5ms pause to signify start of transmission
  if (addr > 0xFF) {          // if extended NEC protocol (16-bit address):
    NEC_sendByte(addr);       // send address low byte
    NEC_sendByte(addr >> 8);  // send address high byte
  } else {                    // if standard NEC protocol (8-bit address):
    NEC_sendByte(addr);       // send address byte
    NEC_sendByte(~addr);      // send inverse of address byte
  }
  NEC_sendByte(cmd);          // send command byte
  NEC_sendByte(~cmd);         // send inverse of command byte
  NEC_normalPulse();          // 562us burst to signify end of transmission
  while(~PINB & BT_MASK) NEC_repeatCode();  // send repeat command until button is released
}

// -----------------------------------------------------------------------------------
// F12 Protocol Implementation (done good enough for a EliteScreens Vmax projector screen)
// The codes are transmitted least significant bit first. Subdevice allowed values are 0 and 1.
// Device values can range from 0-7. Function range 0-255
// -----------------------------------------------------------------------------------

// macros to modulate the signals according to F12 protocol with compensated timings
#define F12_bit0BurstPair()     {IRon(); _delay_us( 420); IRoff(); _delay_us( 1272);}
#define F12_bit1BurstPair()     {IRon(); _delay_us( 1265); IRoff(); _delay_us( 416);}
#define F12_repeatDelay()       {_delay_us(33760);}

  // send complete telegram (address + command) via IR
void F12_sendCode(uint8_t device, uint8_t subdevice, uint8_t function) {
  // prepare carrier wave
  OCR0A  = NEC_TOP;           // set PWM frequency same as NEC
  OCR0B  = NEC_DUTY;          // set duty cycle  same as NEC

  // Combine device and subdevice into a single byte
  uint8_t addr = (subdevice << 3) | (device & 0b111);

  // Make copies to reset addr and function in the loops
  uint8_t addr_original{addr}, function_original{function};
  uint8_t i{2};
  //while (~PINB & BT_MASK || i > 0) {
  while (i > 0) {

    // Send the combined address byte
    for (uint8_t j = 4; j; --j, addr >>= 1) {  // send 4 bits, LSB first
      if (addr & 1) {
        F12_bit1BurstPair();
      } else {
        F12_bit0BurstPair();
      }
    }

    // Send the function byte
    for (uint8_t j = 8; j; --j, function >>= 1) {  // send 8 bits, LSB first
      if (function & 1) {
        F12_bit1BurstPair();
      } else {
        F12_bit0BurstPair();
      }
    }
    F12_repeatDelay();
    addr = addr_original;
    function = function_original;
    --i;
  }
  // End of transmission
}


// -----------------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------------

// main function
int main(void) {
  // set oscillator calibration value
  #ifdef OSCCAL_VAL
    OSCCAL = OSCCAL_VAL;                // set the value if defined above
  #endif

  // setup pins
  DDRB  = 0b00000000;                   // all pins are input pins by now
  PORTB = BT_MASK;                      // pull-up for button pins
  
  // set timer0 to toggle IR pin at 38 kHz
  TCCR0A = 0b00100011;                  // PWM on OC0B (PB1)
  TCCR0B = 0b00001001;                  // no prescaler

  // setup pin change interrupt
  GIMSK = 0b00100000;                   // turn on pin change interrupts
  PCMSK = BT_MASK;                      // turn on interrupt on button pins
  SREG |= 0b10000000;                   // enable global interrupts

  // disable unused peripherals and set sleep mode to save power
  ADCSRA = 0b00000000;                  // disable ADC
  ACSR   = 0b10000000;                  // disable analog comperator
  PRR    = 0b00000001;                  // shut down ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down

  // main loop
  while(1) {
    sleep_mode();                       // sleep until button is pressed
    _delay_ms(1);                       // debounce
    uint8_t buttons = ~PINB & BT_MASK;  // read button pins
    switch (buttons) {                  // send corresponding IR code
      case 0b00000001: KEY1; break;
      case 0b00000100: KEY2; break;
      case 0b00001000: KEY3; break;
      case 0b00010000: KEY4; break;
      case 0b00100000: KEY5; break;
      default: break;
    }
  }
}

// pin change interrupt service routine
EMPTY_INTERRUPT (PCINT0_vect);                // nothing to be done here, just wake up from sleep
