/*
  PS2Communication.h - PS2Communication library
  Copyright (c) 2009 Free Software Foundation.  All right reserved.
  Rewritten for interrupt and ported for Spark Core
  by Andreas RothenwÃ¤nder (aka ScruffR)
  based on some non-interrupt library from pjrc.com (Paul Stoffregen)

  This library provides the basic PS/2 communication framework for
  sending single byte "commands" and receiving data/responses of a
  PS/2 device.
  One example for communication between Spark Core and a PS/2 mouse is
  provided along with the library.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef PS2Communication_h
#define PS2Communication_h

#if defined(SPARK)
#include "application.h"

// some very useful macros for Spark Core and porting Arduino libraries for it

#if (SYSTEM_VERSION < 0x00040400) // no fast pin functions before 0.4.4
 #if defined(STM32F2XX)  // for the Photon and friends
  STM32_Pin_Info* PIN_MAP_= HAL_Pin_Map();
  #define pinResetFast(_pin)	               (PIN_MAP_[_pin].gpio_peripheral->BSRRH = PIN_MAP_[_pin].gpio_pin)
  #define pinSetFast(_pin)	                 (PIN_MAP_[_pin].gpio_peripheral->BSRRL = PIN_MAP_[_pin].gpio_pin)
 #elif (STM32F10X)  // for the Core
  STM32_Pin_Info* PIN_MAP_ = PIN_MAP[_pin];
  #define pinResetFast(_pin)	               (PIN_MAP_[_pin].gpio_peripheral->BRR = PIN_MAP_[_pin].gpio_pin)
  #define pinSetFast(_pin)	                 (PIN_MAP_[_pin].gpio_peripheral->BSRR = PIN_MAP_[_pin].gpio_pin)
 #endif
 #define digitalWriteFast(_pin, _hilo)      (_hilo ? pinSetFast(_pin) : pinResetFast(_pin))
 #define pinReadFast(_pin)                  (PIN_MAP_[_pin].gpio_peripheral->IDR & PIN_MAP_[_pin].gpio_pin ? 0xFF : LOW)
 #define digitalPinToBitMask(_pin)          (PIN_MAP_[_pin].gpio_pin)
 #define digitalPinToPort(_pin)             (PIN_MAP_[_pin].gpio_peripheral)
#endif

// even faster port based multi pin access
#define portSet(_port, _word)              (_port->ODR = _word)
#define portSetMasked(_port, _word, _mask) (_port->BSRR = (_mask << 16) | (_word & _mask))

// Arduino porting/replacement macros
#define pgm_read_byte(_addr)               (*(const uint8_t *)(_addr))
#define pgm_read_byte_near(_addr)          (pgm_read_byte(_addr))
#define pgm_read_word(_addr)               (*(const uint16_t *)(_addr))
#define pgm_read_word_near(_addr)          (pgm_read_word(_addr))
#define portInputRegister(_port)           (_port->IDR)
#define portOutputRegister(_port)          (_port->ODR)
#define cbi(_pin)                          pinResetFast(_pin)
#define sbi(_pin)                          pinSetFast(_pin)
#define bitRead(_val, _bit)                (_val & (1 << _bit))
#define bitWrite(_dest, _bit, _src)        (_dest |= (_src ? (1 << _bit) : 0))

// default values
#define PS2_DATAPIN D0        // needs to be 5V toletant
                              // on the Core   D0, D1, D3, D4, D5, D6 and D7
                              // on the Photon all but A3 and A6/DAC
#define PS2_CLKPIN  D1        // needs to be 5V tolerant & interrupt enabled
                              // on the Core   D0, D1, D3 and D4
                              // on the Photon all but D0, A3, A5, A6/DAC and only one of 
                              //     (D1,A4), (D2,A0,<A3>), (D3,<DAC>), (D4,A1) at a time

// interrupt for PS/2 communication
#define PS2_INTERRUPT PS2_CLKPIN

#else
#include "WProgram.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#define digitalWriteFast(_pin, _hilo)    digitalWrite(_pin, _hilo)
#define pinReadFast(_pin)                digitalRead(_pin)

// default PIN for pin change interrupt PCINT0
//#define PS2_CLKPIN 3        // for arduino
//#define _dataPin 10
#define PS2_CLKPIN  PIN_D1  // Teensy 1.0
#define PS2_DATAPIN PIN_B0  // Teensy 1.0

// interrupt for PS/2 communication
#define PS2_INTERRUPT 1

#endif

#define PS2BUFFER     64
#define WAIT4PS2REPLY  3            // time to transmit a PS2 "byte" [ms]

// direction of communication
#define DEV2HOST 0
#define HOST2DEV -1

extern uint8_t _dataPin;
extern uint8_t _clkPin;
extern uint8_t _clkInterrupt;

extern volatile uint8_t ps2InBufferHead;
extern volatile uint8_t ps2InBufferTail;
extern volatile uint16_t inByte, outByte;

class PS2Communication
{
  private:
    // sets the line states for clock or data
    //   high .. pin becomes input with pullup resistor
    //   low  .. pin becomes output LOW
    inline void setPin(int pin, uint8_t state);
    // in case of tight memory, squeeze out some bytes ;-)
    //void setPin(int pin, uint8_t state);

  public:
    // constructor does the pin setup and attaches the interrupt
    //   for Spark Core direct connection
    //   pins need to be 5V tolerant & clkPin needs to be interrupt enabled
    //   5V toletant D0, D1, D3, D4, D5, D6 and D7
    //   of which D0, D1, D3 and D4 also support HW interrupts
    //   with level shifting all GPIO pins can be used for data and
    //   D0, D1, D2, D3, D4, A0, A1, A3, A4, A5, A6 and A7 for clock
#if defined(SPARK)
  PS2Communication(uint8_t dataPin      = PS2_DATAPIN,
                   uint8_t clkPin       = PS2_CLKPIN);
#else
  	PS2Communication(uint8_t dataPin      = PS2_DATAPIN,
                     uint8_t clkPin       = PS2_CLKPIN,
                     uint8_t clkInterrupt = PS2_INTERRUPT);
#endif

    // does the pin setup and attaches the interrupt via reset()
    void begin();

    // indicates whether there is any new data (TRUE) or not (FALSE)
    uint8_t available();

    // returns the next FIFO byte read of the PS2 device
    uint8_t read();

    // sends one byte to the PS/2 device
    void write(uint8_t data);

    // flush the in/out buffers
    void flush();
    void flush(int buffer);

    // resets the PS/2 device by pulling clock and data low
    void reset();

    // suspends communication by pulling clock low
    void suspend();

    // resumes communication by releasing clock line
    void resume();

    // host requests to send data to the device
    void rts();
};
#endif
