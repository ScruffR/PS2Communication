/*
  PS2Communication.h - PS2Communication library
  Copyright (c) 2009 Free Software Foundation.  All right reserved.
  Rewritten for interrupt and ported for Spark Core
  by Andreas Rothenwänder <scruff.r@sbg.at>
  based on some non-interrupt library from pjrc.com (Paul Stoffregen)

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
volatile uint8_t ps2InBuffer[32];
volatile uint8_t ps2InBufferHead;
volatile uint8_t ps2InBufferTail;
volatile uint8_t ps2OutByte;
volatile uint8_t ps2IgnoreResponse;

volatile uint8_t state = 0;

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
#if defined(SPARK)
  _clkInterrupt = clkPin;
#else
  _clkInterrupt = clkInterrupt;
#endif

  // initialize the pins
  PS2Communication::reset();
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
  delayMicroseconds(10000);             // long enough for a reset?
  PS2Communication::setPin(_dataPin, HIGH);
  PS2Communication::resume();
  PS2Communication::write(0xFF);
  delay(500);
}

uint8_t PS2Communication::available()
{
  return ps2InBufferHead - ps2InBufferTail;
}

uint8_t PS2Communication::read()
{
  if (ps2InBufferHead > ps2InBufferTail)
    return ps2InBuffer[ps2InBufferTail++];
  else
    return 0;
}

void PS2Communication::write(uint8_t data, uint8_t ignoreResponse)
{
  ps2OutByte = data;
  ps2IgnoreResponse = ignoreResponse;
  PS2Communication::rts();                  // request to send
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
{
  if (state)
  {
#if defined(SPARK)
    pinMode(pin, INPUT_PULLUP);
#else
    pinMode(pin, INPUT);
    pinSet(pin, HIGH);
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
    pinSet(pin, LOW);
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
        pinSet(_dataPin, HIGH);
        _Parity ^= 0xFF;                     // toggle parity
      }
      else
        pinSet(_dataPin, LOW);
      break;
    case 9:   // parity
      pinSet(_dataPin, _Parity);    // send parity bit
      break;
    case 10:  // stopbit
      pinSet(_dataPin, HIGH);
      break;
    case 11: // only for HOST2DEV: acknowledge sent by device
      detachInterrupt(_clkInterrupt);

      //if (!pinGet(_dataPin))      // if sent byte is acknowledged remove it from the buffer
      //  ps2OutByte = 0;
      ps2OutByte = 0;
      ps2BitPos = -1;

      if (ps2IgnoreResponse)
      { // abort transmission
        pinLO(_clkPin);
        delayMicroseconds(100);
        pinHI(_clkPin);
        delayMicroseconds(25);
        pinMode(_clkPin, INPUT);
      }
      else
      {
        pinMode(_dataPin, INPUT);

        ps2Direction = DEV2HOST;                // allow device to host communication again (e.g. for command result transfer)

        attachInterrupt(_clkInterrupt, ps2DeviceToHostCommunication, FALLING);
      }
      break;
    default:
      //pinSet(_dataPin, HIGH);
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
      if (pinGet(_dataPin))
      {
        _Data |= (0x01 << (ps2BitPos - 1));
        _Parity ^= 0xFF;                     // toggle parity
      }
      break;
    case 9:   // parity
      _Parity ^= pinGet(_dataPin);   // if parity bit does meet the expectation ps2Parity is cleared

      // if we don't care for parity and stopbit - do it now
      if (ps2InBufferHead && ps2InBufferTail == ps2InBufferHead)
        ps2InBufferTail = ps2InBufferHead = 0;

      //if (!_Parity)  // don't care about parity ;-)
      ps2InBuffer[ps2InBufferHead++] = _Data;

      break;
    case 10:  // stopbit
      //// if we do care - wait for the stopbit and check parity
      //if (pinGet(_dataPin) && !_Parity)
      //{
      //  if (ps2InBufferHead && ps2InBufferTail == ps2InBufferHead)
      //    ps2InBufferTail = ps2InBufferHead = 0;
      //
      //  ps2InBuffer[ps2InBufferHead++] = _Data;
      //}
    default:
      ps2BitPos = -1;
  }
  ps2BitPos++;
}

/* superceded by dedicated ISR depending on direction
volatile uint8_t  ps2InByte;        // first version vars (can go as soon as ps2Interrupt is not needed anymore)
volatile uint8_t  ps2Parity;        //  --"--
volatile uint16_t inByte, outByte;  //  --"--

void PS2Communication::ps2FirstRising(void)
{
  ps2BitPos = 0;
  attachInterrupt(_clkInterrupt, ps2Direction == DEV2HOST ? ps2DeviceToHostCommunication : ps2HostToDeviceCommunication, FALLING);
}

// The ISR for the PS/2 clock falling edge
void ps2Interrupt(void)
{
  uint8_t bit;

  switch (ps2BitPos)
  {
  case 0:   // startbit
    break;
  case 1:   // 8 databits
    ps2Parity = 0xFF;       // preset parity flag
    if (ps2Direction == HOST2DEV)
    {
      pinMode(_dataPin, OUTPUT);
      outByte = 0x8000;
    }
    else
    {
      ps2InByte = 0;
      inByte = 0x8000;
    }
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
  case 7:
  case 8:
  {
    if (ps2Direction == DEV2HOST)
    {
      if (bit = digitalRead(_dataPin))
      {
        ps2InByte |= (0x01 << (ps2BitPos - 1));
        ps2Parity = ~ps2Parity;
        inByte |= (0x01 << (ps2BitPos - 1));
        inByte ^= 0x8000;
      }
    }
    else //if (ps2Direction == HOST2DEV)
    {
      if (ps2OutByte & (0x01 << (ps2BitPos - 1)))
      {
        pinSet(_dataPin, HIGH);
        ps2Parity = ~ps2Parity;
        outByte |= (0x01 << (ps2BitPos - 1));
      }
      else
      {
        pinSet(_dataPin, LOW);
        outByte &= ~(0x01 << (ps2BitPos - 1));
      }
    }

    break;
  }
  case 9:   // parity
  {
    if (ps2Direction == DEV2HOST)
    {
      inByte |= (ps2Parity << 15);
      ps2Parity ^= digitalRead(_dataPin);   // if parity bit does meet the expectation ps2Parity is cleared
    }
    else //if (ps2Direction == HOST2DEV)
    {
      outByte |= (ps2Parity << 14);
      pinSet(_dataPin, ps2Parity);    // send parity bit
    }
    break;
  }
  case 10:  // stopbit
  {
    if (ps2Direction == DEV2HOST)
    {
      if (ps2InBufferHead && ps2InBufferTail == ps2InBufferHead)
        ps2InBufferTail = ps2InBufferHead = 0;

      //        if (!ps2Parity)
      ps2InBuffer[ps2InBufferHead++] = ps2InByte;

      ps2BitPos = -1;
    }
    else //if (ps2Direction == HOST2DEV)
    {
      #if defined(SPARK)
      pinMode(_dataPin, INPUT_PULLUP);
      #else
      pinMode(_dataPin, INPUT);
      pinSet(_dataPin, HIGH);
      #endif
    }
    break;
  }
  case 11: // only for HOST2DEV: acknowledge sent by device
  {
    //      if (digitalRead(_dataPin) == LOW)      // if sent byte is acknowledged remove it from the buffer
    //        ps2OutByte = 0;

    ps2BitPos = -1;
    ps2Direction = DEV2HOST;                // allow device to host communication again (e.g. for command result transfer)
    break;
  }
  default:
    ps2OutByte =
      ps2InByte = 0;
    ps2BitPos = -1;
    ps2Direction = DEV2HOST;                // allow device to host communication again (e.g. for command result transfer)
    #if defined(SPARK)
    pinMode(_dataPin, INPUT_PULLUP);
    #else
    pinMode(_dataPin, INPUT);
    pinSet(_dataPin, HIGH);
    #endif
    break;
  }
  ps2BitPos++;
}
*/
