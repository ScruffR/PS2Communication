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

#include "PS2Communication.h"

// ISR prototypes
//void ps2FirstRising(void);
void ps2HostToDeviceCommunication(void);
void ps2DeviceToHostCommunication(void);

uint8_t _dataPin;
uint8_t _clkPin;
uint8_t _clkInterrupt;

volatile int8_t  ps2Direction = DEV2HOST;
volatile int8_t  ps2BitPos;          // shared between ISRs for DEV2HOST & HOST2DEV communication
volatile uint8_t ps2InBuffer[PS2BUFFER];
volatile uint8_t ps2InBufferHead;
volatile uint8_t ps2InBufferTail;
volatile uint8_t ps2OutByte;

//volatile uint8_t ps2State = 0;

#if defined(SPARK)
PS2Communication::PS2Communication(uint8_t dataPin,
                                   uint8_t clkPin)
#else
PS2Communication::PS2Communication(uint8_t dataPin,
                                   uint8_t clkPin,
                                   uint8_t clkInterrupt)
#endif
{
  // initialize class variables
  _dataPin      = dataPin;
  _clkPin       = clkPin;
#if defined(PARTICLE)
  _clkInterrupt = clkPin;
#else
  _clkInterrupt = clkInterrupt;
#endif
}

void PS2Communication::begin()
{
  PS2Communication::reset();
}

void PS2Communication::reset()
{
  PS2Communication::suspend();
  PS2Communication::setPin(_dataPin, LOW);
  PS2Communication::flush();
  delayMicroseconds(100000);             // 100ms should suffice
  PS2Communication::setPin(_dataPin, HIGH);
  PS2Communication::resume();
  PS2Communication::write(0xFF);
  delay(500);
}

uint8_t PS2Communication::available()
{
	return (PS2BUFFER + ps2InBufferHead - ps2InBufferTail) % PS2BUFFER;
}

uint8_t PS2Communication::read()
{
  uint8_t r = 0;

	if (ps2InBufferHead != ps2InBufferTail)
	{
		r = ps2InBuffer[ps2InBufferTail++];
		ps2InBufferTail %= PS2BUFFER;
	}

  return r;
}

void PS2Communication::write(uint8_t data)
{
  ps2OutByte = data;
  PS2Communication::rts();                  // request to send
  delay(WAIT4PS2REPLY);
}

//void PS2Communication::write(uint8_t data)
//{
//  PS2Communication::write(data, false);
//}

void PS2Communication::flush()
{
  ps2OutByte =
  ps2InBufferHead  =
  ps2InBufferTail  = 0;
}

void PS2Communication::flush(int buffer)
{
  if (buffer == DEV2HOST)
    ps2InBufferHead = ps2InBufferTail = 0;
  else
    ps2OutByte = 0;
}

void PS2Communication::suspend()
{
  PS2Communication::setPin(_clkPin, LOW);
  delayMicroseconds(100);   // at least 100 micro seconds
}

void PS2Communication::resume()
{
  // immediatly
  //ps2InByte =
  ps2BitPos =
  ps2OutByte = 0;
  PS2Communication::setPin(_clkPin, HIGH);
}

void PS2Communication::rts()
{
  PS2Communication::setPin(_clkPin, LOW);
  delayMicroseconds(100);
  PS2Communication::setPin(_dataPin, LOW); // produce startbit
  delayMicroseconds(25);
  ps2BitPos = 0;
  ps2Direction = HOST2DEV;
  PS2Communication::setPin(_clkPin, HIGH);
}

inline void PS2Communication::setPin(int pin, uint8_t state)
// in case of tight memory, squeeze out some bytes ;-)
//void setPin(int pin, uint8_t state);
{
  if (state)
  {
#if defined(PARTICLE)
    pinMode(pin, INPUT_PULLUP);
#else
    pinMode(pin, INPUT);
    digitalWriteFast(pin, HIGH);
#endif
    if (pin == _clkPin)
    {
      ps2BitPos = 0;
      attachInterrupt(_clkInterrupt, (ps2Direction == DEV2HOST)
                                     ? ps2DeviceToHostCommunication
                                     : ps2HostToDeviceCommunication
                                     , FALLING);
    }
  }
  else
  {
    if (pin == _clkPin)
      detachInterrupt(_clkInterrupt);

    pinMode(pin, OUTPUT);
    digitalWriteFast(pin, LOW);
  }
}

void ps2HostToDeviceCommunication(void)
{
  static uint8_t _Data;
  static uint8_t _Parity;

  switch (ps2BitPos)
  {
    case 0:             // startbit
      _Data = ps2OutByte;
      _Parity = 0xFF;
      ps2BitPos++;      // since startbit is already set by RTS function
    case 1:             // 8 databits
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
      if (_Data & (0x01 << (ps2BitPos - 1)))
      {
        digitalWriteFast(_dataPin, HIGH);
        _Parity ^= 0xFF;                     // toggle parity
      }
      else
        digitalWriteFast(_dataPin, LOW);
      break;
    case 9:   // parity
      digitalWriteFast(_dataPin, _Parity);    // send parity bit
      break;
    case 10:  // stopbit
      digitalWriteFast(_dataPin, HIGH);
      break;
    case 11: // only for HOST2DEV: acknowledge sent by device
      detachInterrupt(_clkInterrupt);

      //if (!pinReadFast(_dataPin))      // if sent byte is acknowledged remove it from the buffer
      //  ps2OutByte = 0;
      ps2OutByte = 0;
      ps2BitPos = -1;

      /* experimental
      if (ps2IgnoreResponse)
      { // abort transmission
        pinResetFast(_clkPin);
        delayMicroseconds(100);
        pinSetFast(_clkPin);
        delayMicroseconds(25);
        pinMode(_clkPin, INPUT);
      }
      else
      */
      {
        pinMode(_dataPin, INPUT);

        ps2Direction = DEV2HOST;                // allow device to host communication again (e.g. for command result transfer)

        attachInterrupt(_clkInterrupt, ps2DeviceToHostCommunication, FALLING);
      }
      break;
    default:
      //digitalWriteFast(_dataPin, HIGH);
      pinMode(_dataPin, INPUT);
      ps2BitPos = -1;
      detachInterrupt(_clkInterrupt);
  }
  ps2BitPos++;
}

void ps2DeviceToHostCommunication(void)
{
  static uint8_t _Data;
  static uint8_t _Parity;

  switch (ps2BitPos)
  {
    case 0:   // startbit
      _Data = 0;
      _Parity = 0xFF;
      break;
    case 1:   // 8 databits
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
      if (pinReadFast(_dataPin))
      {
        _Data |= (0x01 << (ps2BitPos - 1));
        _Parity ^= 0xFF;                     // toggle parity
      }
      break;
    case 9:   // parity
      _Parity ^= pinReadFast(_dataPin);   // if parity bit does meet the expectation ps2Parity is cleared

      // if we don't care for parity and stopbit - do it now

      //if (!_Parity)  // don't care about parity ;-)
      {
        uint8_t i = (ps2InBufferHead + 1) % PS2BUFFER;
	      if (i != ps2InBufferTail)
	      {
		      ps2InBuffer[ps2InBufferHead] = _Data;
		      ps2InBufferHead = i;
	      }
      }
      break;
    case 10:  // stopbit
      //// if we do care - wait for the stopbit and check parity
      //if (pinReadFast(_dataPin) && !_Parity)
      //{
      //  uint8_t i = (ps2InBufferHead + 1) % PS2BUFFER;
      //  if (i != ps2InBufferTail)
      //  {
      //    ps2InBuffer[ps2InBufferHead] = _Data;
      //    ps2InBufferHead = i;
      //  }
      //}
    default:
      ps2BitPos = -1;
  }
  ps2BitPos++;
}
