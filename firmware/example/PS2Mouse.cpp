/*
  PS2Mouse.cpp - read PS/2 mouse input (interrupt driven)
  Copyright (c) 2009 Andreas Rothenwänder.  All right reserved.
  Written by Andreas Rothenwänder <scruff.r@sbg.at>

  This firmware reads PS/2 mouse reports and forwards that information via
  Serial output.

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

#include "PS2Communication/PS2Communication.h"

SYSTEM_MODE(MANUAL);

#define DEBUG_PS2
#define DEBUG_DO_MOUSE

#define REPEATTIME    100
#define BUFLEN         32

PS2Communication* PS2 = NULL;
uint8_t ps2DeviceID = 0;

char buffer[BUFLEN];

unsigned long ps2LastRead;
char lastMouseState = 0xFF;

// function prototypes
void ps2MouseInit();
void ps2MouseRead();


uint8_t bEcho = FALSE;
char echo[8] = {'\0'};
int  echoLen = 0;
long l;

void setup()
{
  pinMode(D7, OUTPUT);

  memset(buffer, 0, BUFLEN);            // clear buffer

  PS2 = new PS2Communication();

#if defined(DEBUG_PS2) || defined(DEBUG_DO_MOUSE)
   // USB bridge for debug information
   Serial.begin(115200);

  ps2LastRead = millis();
  while(!Serial.available() && millis() - ps2LastRead < 10000)
    SPARK_WLAN_Loop();
#endif

  ps2MouseInit();

  ps2LastRead = millis();
}

void loop()
{
  if (!bEcho)
  {
    if (millis() - ps2LastRead >= REPEATTIME)
    {
      ps2MouseRead();
      ps2LastRead = millis();
    }
  }

  while (Serial.available())
    Serial.write(echo[echoLen++] = Serial.read());

  echo[echoLen] = '\0';

  if (echoLen == 7 || echo[echoLen-1] == '\n' || echo[echoLen-1] == '\r')
  {
    Serial.println(echo);
    Serial.println("-----");
    echoLen = 0;

    if (echo[0] == 'X')
    {                       // enter dfu
      delay(5000);          // give us some time to close serial monitor
      FLASH_OTA_Update_SysFlag = 0x0000;
      Save_SystemFlags();
      BKP_WriteBackupRegister(BKP_DR10, 0x0000);
      USB_Cable_Config(DISABLE);
      NVIC_SystemReset();
    }

    l = strtol(echo, NULL, 16);
    if (l > 0 && !bEcho)
    {
      PS2->write(0xEE);    // enable  Wrap Mode (echo back each byte from host)
      //PS2->write(0xEC);    // disable Wrap Mode (echo back each byte from host)
      delay(WAIT4PS2REPLY);
      bEcho = TRUE;
    }
    else if (l == 0xFF) // || 0xEC)
    {
      bEcho = FALSE;
      l = 0;
      ps2MouseInit();
      return;
    }

    if (l > 0)
    {
      PS2->write((byte)l);
      delay(WAIT4PS2REPLY);
      while (PS2->available())
      {
        Serial.println(PS2->read(), HEX);
        delay(WAIT4PS2REPLY);
      }
      Serial.println("^^^^^");
    }
  }
}

inline void dump()
{
  /*
  uint8_t r;

  while (PS2->available())
  {
    if (r = PS2->read())
      Serial.println(r, HEX);
    else
      Serial.println("NULL");
  }
  Serial.println("-----");
  */
}

void ps2MouseInit()
{
  PS2->reset();
  delay(5 * WAIT4PS2REPLY);   // wait reset to finish

Serial.println("5button");
  // try to set as 5 button scroll mouse (magic sample rate sequence 200, 200, 80)
  PS2->write(0xF3);           // set sample rate
  delay(2 * WAIT4PS2REPLY);   // allow time for reply (1 send + 1 receive)
dump();
  PS2->write(200);  // 0xC8
  delay(2 * WAIT4PS2REPLY);   // allow time for reply
dump();
  PS2->write(0xF3);
  delay(2 * WAIT4PS2REPLY);   // allow time for reply
dump();
  PS2->write(200);  // 0xC8
  delay(2 * WAIT4PS2REPLY);   // allow time for reply
dump();
  PS2->write(0xF3);
  delay(2 * WAIT4PS2REPLY);   // allow time for reply
dump();
  PS2->write(80);   // 0x50
  delay(2 * WAIT4PS2REPLY);   // allow time for reply
dump();
  PS2->flush();

  PS2->write(0xF2);           // now get device ID
  delay(3 * WAIT4PS2REPLY);   // allow time for reply ( 1 send + 2 receive)
  PS2->read();                // drop ACK
  ps2DeviceID = PS2->read();

  if (ps2DeviceID != 0x04)
  {
    PS2->flush();

Serial.println("3button");
    // try to set as 3 button scroll mouse (magic sample rate sequence 200, 100, 80)
    PS2->write(0xF3);           // set sample rate
    delay(2 * WAIT4PS2REPLY);   // allow time for reply
dump();
    PS2->write(200);            // 0x200
    delay(2 * WAIT4PS2REPLY);   // allow time for reply
dump();
    PS2->write(0xF3);
    delay(2 * WAIT4PS2REPLY);   // allow time for reply
dump();
    PS2->write(100);            // 0x64
    delay(2 * WAIT4PS2REPLY);   // allow time for reply
dump();
    PS2->write(0xF3);
    delay(2 * WAIT4PS2REPLY);   // allow time for reply
dump();
    PS2->write(80);             // 0x50
    delay(2 * WAIT4PS2REPLY);   // allow time for reply
dump();
    PS2->flush();               // drop all ACKs

    PS2->write(0xF2);           // now get device ID
    delay(3 * WAIT4PS2REPLY);   // allow time for reply
    PS2->read();                // drop ACK
    ps2DeviceID = PS2->read();
  }

#ifdef DEBUG_PS2
  switch (ps2DeviceID)
  {
    case 0x00:
      Serial.println("2 button standard mouse");
      break;
    case 0x03:
      Serial.println("3 button scroll mouse");
      break;
    case 0x04:
      Serial.println("5 button scroll mouse");
      break;
    default:
      Serial.print(ps2DeviceID, HEX);
      Serial.println(" - not a valid mouse type");
      ps2DeviceID = 0x00;
      break;
  }
#endif

  delay(WAIT4PS2REPLY);
  PS2->flush();        // drop all ACKs

  PS2->write(0xF3);    // set sample rate
  delay(2 * WAIT4PS2REPLY);
dump();
  PS2->write(50);      // 50 samples per second
  delay(2 * WAIT4PS2REPLY);
dump();
  PS2->write(0xE8);    // set resolution
  delay(2 * WAIT4PS2REPLY);
dump();
  PS2->write(0x02);    // 4 counts per mm
  delay(2 * WAIT4PS2REPLY);
dump();

  //PS2->write(0xEA);    // set stream mode (should be default anyway)
  //delay(WAIT4PS2REPLY);
  //PS2->write(0xF4);    // enable reporting for stream mode
  //delay(WAIT4PS2REPLY);

  PS2->write(0xF0);    // remote mode (mouse needs to be asked for data)
  delay(2 * WAIT4PS2REPLY);
dump();

  //PS2->write(0xEE);    // enable  Wrap Mode (echo back each byte from host)
  //delay(2 * WAIT4PS2REPLY);
  //PS2->write(0xEC);    // disable Wrap Mode (echo back each byte from host)
  //delay(2 * WAIT4PS2REPLY);

  PS2->flush();
}

void ps2MouseRead()
{
  uint8_t mState = 0x00;
  int16_t  mX = 0x00;
  int16_t  mY = 0x00;
  int8_t  mZ = 0x00;

  PS2->flush();

  // get a mouse report
  PS2->write(0xEB);          // give me data!
  delay(5 * WAIT4PS2REPLY);
  if (PS2->available())
  {
    PS2->read();            // drop ACK (0xFA)
    // see http://www.computer-engineering.org/ps2mouse/
    mState = PS2->read();
    mX = (mState & 0x10 ? 0xF0 : 0x00) | PS2->read();
    mY = (mState & 0x20 ? 0xF0 : 0x00) | PS2->read();

    if (ps2DeviceID == 0x03 || ps2DeviceID == 0x04)
    {
      delay(WAIT4PS2REPLY);
      mZ = PS2->read();
      mZ |= (mZ & 0x80) ? 0xF0 : 0x00;  // expand sign & forget other info
      // more info at http://www.computer-engineering.org/ps2mouse/
    }

    // with USB_HID_Mouse this could be passed on to a computer
    //if (mX || mY || (mZ & 0x0F))
    //  Mouse.move(mX, -mY, -mZ);

    if (mState != lastMouseState)
    {
      lastMouseState = mState;
      //Mouse.set_buttons(mState & 0x01, mState & 0x04, mState & 0x02);
    }

#ifdef DEBUG_PS2
    if (mX != 0 || mY != 0 || mZ != 0 || mState != lastMouseState)
    {
      // send the data back up
      Serial.println();
      Serial.print(mState & 0x07, BIN);
      Serial.print("\tX=");
      Serial.print(mX, DEC);
      Serial.print("\tY=");
      Serial.print(mY, DEC);
      Serial.print("\tZ=");
      Serial.print(mZ, DEC);
      Serial.println();
    }
#endif
  }
  else
  {
    PS2->flush();
#ifdef DEBUG_PS2
    Serial.print(".");
#endif
  }
}
