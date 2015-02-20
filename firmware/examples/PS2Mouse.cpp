/*
  PS2Mouse.cpp - read PS/2 mouse input (interrupt driven)
  Copyright (c) 2009 Andreas Rothenwänder.  All right reserved.
  Written by Andreas Rothenwänder (aka ScruffR)

  This firmware reads PS/2 mouse reports and forwards that information via
  Serial output and passes out cummulated values as Spark.variables

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

//SYSTEM_MODE(MANUAL);

#define xDEBUG_PS2
#define DEBUG_DO_MOUSE

#if defined DEBUG_PS2
#define PS2IGNORE FALSE
#else
#define PS2IGNORE TRUE
#endif

#define REPEATTIME     100
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

// mouse values for Spark.variables
char mouseB[] = "_lmr_";
int mouseX = 0x00;
int mouseY = 0x00;
int mouseZ = 0x00;
int mouseReset(String cmd);

void setup()
{
  Spark.variable("MouseButtons", mouseB, STRING);
  Spark.variable("MouseX", &mouseX, INT);
  Spark.variable("MouseY", &mouseY, INT);
  Spark.variable("MouseZ", &mouseZ, INT);
  Spark.function("MouseReset", mouseReset);

  memset(buffer, 0, BUFLEN);            // clear buffer

#if defined(DEBUG_PS2) || defined(DEBUG_DO_MOUSE)
   // USB bridge for debug information
   Serial.begin(115200);

  ps2LastRead = millis();
  while(!Serial.available() && millis() - ps2LastRead < 10000)
    SPARK_WLAN_Loop();
#else
  delay(2000); // time for PS/2 mouse to settle in
#endif

  // dataPin D0, clkPin D1
  PS2 = new PS2Communication(D0, D1);

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

#if defined(DEBUG_PS2) || defined(DEBUG_DO_MOUSE)
  // here you can send test bytes to the PS/2 mouse, which will be echoed
  // back (except 0xFF .. reset & 0xEC .. end echo for response test)
  // or enter X to switch into dfu-mode

  while (Serial.available())
    Serial.write(echo[echoLen++] = Serial.read());
  echo[echoLen] = '\0';

  if (echoLen == 7 || echo[echoLen-1] == '\n' || echo[echoLen-1] == '\r')
  {
    Serial.println(echo);
    Serial.println("-----");
    echoLen = 0;

    // Beware! You need to flash firmware via USB (dfu-util) after this
    if (echo[0] == 'X')
    {                        // enter dfu
      delay(5000);           // give us some time to close serial monitor
      FLASH_OTA_Update_SysFlag = 0x0000;
      Save_SystemFlags();
      BKP_WriteBackupRegister(BKP_DR10, 0x0000);
      USB_Cable_Config(DISABLE);
      NVIC_SystemReset();
    }

    l = strtol(echo, NULL, 16);
    if (l > 0 && !bEcho)
    {
      PS2->write(0xEE);      // enable  Wrap Mode (echo back each byte from host)
      //PS2->write(0xEC);    // disable Wrap Mode (echo back each byte from host)
      delay(WAIT4PS2REPLY);
      bEcho = TRUE;
    }
    else if (l == 0xFF)
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
#endif
}

void dumpACK()
{
  delay(WAIT4PS2REPLY);

#if defined(DEBUG_PS2)
  uint8_t r;

  delay(WAIT4PS2REPLY);
  while (PS2->available())
  {
    delay(WAIT4PS2REPLY);
    if (r = PS2->read())
      Serial.println(r, HEX);
    else
      Serial.println("NULL");
  }
  Serial.println("-----");
#endif
}

void ps2MouseInit()
{
  PS2->reset();
  delay(5 * WAIT4PS2REPLY);  // wait reset to finish

Serial.println("5button");
  // try to set as 5 button scroll mouse (magic sample rate sequence 200, 200, 80)
  PS2->write(0xF3, PS2IGNORE);   // set sample rate
  dumpACK();                     // allow time for reply (1 send + 1 receive)
  PS2->write(200, PS2IGNORE);    // 0xC8
  dumpACK();
  PS2->write(0xF3, PS2IGNORE);
  dumpACK();
  PS2->write(200, PS2IGNORE);    // 0xC8
  dumpACK();
  PS2->write(0xF3, PS2IGNORE);
  dumpACK();
  PS2->write(80, PS2IGNORE);     // 0x50
  dumpACK();
  PS2->flush();

  PS2->write(0xF2);              // now get device ID
  delay(3 * WAIT4PS2REPLY);      // allow time for reply ( 1 send + 2 receive)
  PS2->read();                   // drop ACK
  ps2DeviceID = PS2->read();

  if (ps2DeviceID != 0x04)
  {
    PS2->flush();

Serial.println("3button");
    // try to set as 3 button scroll mouse (magic sample rate sequence 200, 100, 80)
    PS2->write(0xF3, PS2IGNORE); // set sample rate
    dumpACK();                   // allow time for reply (1 send + 1 receive)
    PS2->write(200, PS2IGNORE);  // 0xC8
    dumpACK();
    PS2->write(0xF3, PS2IGNORE);
    dumpACK();
    PS2->write(100, PS2IGNORE);  // 0x64
    dumpACK();
    PS2->write(0xF3, PS2IGNORE);
    dumpACK();
    PS2->write(80, PS2IGNORE);   // 0x50
    dumpACK();
    PS2->flush();                // drop all ACKs

    PS2->write(0xF2);            // now get device ID
    delay(3 * WAIT4PS2REPLY);    // allow time for reply
    PS2->read();                 // drop ACK
    ps2DeviceID = PS2->read();
  }

#ifdef DEBUG_DO_MOUSE
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
  PS2->flush();                  // drop all ACKs

  PS2->write(0xF3, PS2IGNORE);   // set sample rate
  dumpACK();
  PS2->write(50, PS2IGNORE);     // 50 samples per second
  dumpACK();
  PS2->write(0xE8, PS2IGNORE);   // set resolution
  dumpACK();
  PS2->write(0x02, PS2IGNORE);   // 4 counts per mm
  dumpACK();

  //PS2->write(0xEA, PS2IGNORE);   // set stream mode (should be default anyway)
  //dumpACK();
  //PS2->write(0xF4, PS2IGNORE);   // enable reporting for stream mode
  //dumpACK();

  PS2->write(0xF0, PS2IGNORE);   // remote mode (mouse needs to be asked for data)
  dumpACK();

  //PS2->write(0xEE, PS2IGNORE);   // enable  Wrap Mode (echo back each byte from host)
  //dumpACK();
  //PS2->write(0xEC, PS2IGNORE);   // disable Wrap Mode (echo back each byte from host)
  //dumpACK();

  PS2->flush();
}

void ps2MouseRead()
{
  uint8_t  mState  = 0x00;
  int16_t  mX      = 0x00;
  int16_t  mY      = 0x00;
  int8_t   mZ      = 0x00;
  char     cB45    = ' ';  // place holder for button 4 & 5 if available
  uint8_t  cnt     = (ps2DeviceID < 0x03 ? 4 : 5); // how many bytes to expect
  uint32_t timeout = millis();

  PS2->flush();

  // get a mouse report
  PS2->write(0xEB);          // give me data!
  while (PS2->available() < cnt && millis() - timeout < 10 * WAIT4PS2REPLY);
  if (PS2->available() == cnt)
  {
    PS2->read();            // drop ACK (0xFA)
    // see http://www.computer-engineering.org/ps2mouse/
    mState = PS2->read() & 0x3F;  // don't care for carry flags
    // usually this wouldn't be necessary as most mice only report -127..+127
    // but the protocoll allows for -255..+255 plus a carry flag
    // -128/-256 is not allowed
    if (mX = PS2->read())  // if read byte != 0 expand sign otherwise -256 would happen
      mX |= (mState & 0x10 ? 0xFF00 : 0x0000);
    if (mY |= PS2->read()) // if read byte != 0 expand sign otherwise -256 would happen
      mY |= (mState & 0x20 ? 0xFF00 : 0x0000);

    if (ps2DeviceID >= 0x03)
    {
      mZ = PS2->read();
      mState |= (mZ & 0x30) << 2; // place button 4 & 5 in state byte
      mZ |= (mZ & 0x08) ? 0xFFF0 : 0x0000;  // expand sign & forget other info
      // more info at http://www.computer-engineering.org/ps2mouse/
      if (ps2DeviceID == 0x04)
        cB45 = '_';
    }

    mouseX += (int)mX;
    mouseY += (int)mY;
    mouseZ += (int)mZ;
    sprintf(mouseB, "%c%c%c%c%c", mState & 0x80 ? '4' : cB45
                                , mState & 0x01 ? 'L' : 'l'
                                , mState & 0x04 ? 'M' : 'm'
                                , mState & 0x02 ? 'R' : 'r'
                                , mState & 0x40 ? '5' : cB45);

    // with USB_HID_Mouse this could be passed on to a computer
    //if (mX || mY || (mZ & 0x0F))
    //  Mouse.move(mX, -mY, -mZ);

#ifdef DEBUG_DO_MOUSE
    if (mX != 0 || mY != 0 || mZ != 0 || mState != lastMouseState)
    {
      // send the data back up
      Serial.println();
      Serial.print(mouseB);
      Serial.print(" (");
      Serial.print(mState & 0x0F, BIN);
      Serial.print(")\tX=");
      Serial.print(mX, DEC);
      Serial.print("\tY=");
      Serial.print(mY, DEC);
      Serial.print("\tZ=");
      Serial.print(mZ, DEC);
      Serial.println();
    }
#endif

    if (mState != lastMouseState)
    {
      lastMouseState = mState;
      //Mouse.set_buttons(mState & 0x01, mState & 0x04, mState & 0x02);
    }
  }
  else
  {
    PS2->flush();
#ifdef DEBUG_PS2
    Serial.print(".");
#endif
  }
}

int mouseReset(String cmd)
{
  mouseX =
  mouseY =
  mouseZ = 0;

  return 0;
}
