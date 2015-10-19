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
#pragma SPARK_NO_PREPROCESSOR
#include "application.h"

SYSTEM_MODE(SEMI_AUTOMATIC); // cloud connect with L/M/R button pressed at once

#define SPARK_WEB_IDE      // this would be nice to be provided by Spark ;-)
#if defined(SPARK_WEB_IDE)
#include "PS2Communication/PS2Communication.h"
#else
#define noSPARK_USB_MOUSE    // this should be defined externally in USB HID mode
#include "PS2Communication.h"
#endif

#define DEBUG_PS2
#define DEBUG_DO_MOUSE

#define REPEATTIME     25

typedef enum MOUSEMODES
{
  mmStream  = 0xEA,
  mmWrap    = 0xEE,
  mmRemote  = 0xF0,
  mmReset   = 0xFF,
  mmDefault = mmStream
} MouseModes;

typedef enum MOUSESAMPLERATES
{
  ms10      = 10,
  ms20      = 20,
  ms40      = 40,
  ms60      = 60,
  ms80      = 80,
  ms100     = 100,
  ms200     = 200,
  msNone    = 0xFF,
  msDefault = ms100
} MouseSampleRates;

typedef enum MOUSERESOLUTIONS
{
  mrCount1perMM = 0x00,
  mrCount2perMM = 0x01,
  mrCount4perMM = 0x02,
  mrCount8perMM = 0x03,
  mrNone        = 0xFF,
  mrDefault     = mrCount4perMM
} MouseResolutions;

PS2Communication* PS2    = NULL;
int8_t     ps2DeviceID   = mmReset;
MouseModes ps2MouseMode  = mmReset;

// mouse values for Spark.variables
char    mouseB[]       = "_lmr_";
uint8_t lastMouseState = 0x00;
int     mouseX         = 0x00;
int     mouseY         = 0x00;
int     mouseZ         = 0x00;

unsigned long ps2LastRead;    // millis() when PS/2 was read last

// function prototypes
MouseModes ps2MouseInit(MouseModes Mode             = mmDefault
                       ,MouseSampleRates SampleRate = msNone
                       ,MouseResolutions Resolution = mrNone);
void ps2MouseRead();
void ps2ProcessMouseReport();

int  mouseReset(String cmd);  // Spark.function
void connectCloud();

inline void dumpACK();
#if defined(DEBUG_PS2) || defined(DEBUG_DO_MOUSE)
void doDebugging();
#endif

void setup()
{
  WiFi.off();                 // don't need it unless we call connectCloud()

  pinMode(D7, OUTPUT);

  Spark.variable("MouseButtons", mouseB , STRING);
  Spark.variable("MouseX"      , &mouseX, INT   );
  Spark.variable("MouseY"      , &mouseY, INT   );
  Spark.variable("MouseZ"      , &mouseZ, INT   );

  Spark.function("MouseReset", mouseReset);

#if defined(DEBUG_PS2) || defined(DEBUG_DO_MOUSE)
  Serial.begin(115200);
  while(!Serial.available() && millis() < 5000)
    SPARK_WLAN_Loop();
  while (Serial.available())
    Serial.read();
#else
  delay(2000); // time for PS/2 mouse to settle in
#endif

#if defined(SPARK_USB_MOUSE)
  Mouse.begin();
#endif

  // dataPin D0, clkPin D1
  PS2 = new PS2Communication(D0, D1);

  if ((ps2MouseMode = ps2MouseInit(mmDefault)) != mmDefault)
    connectCloud();  // without PS/2 mouse you'll need the cloud ;-)

  ps2LastRead = millis();
}

void loop()
{
  if ((lastMouseState & 0x07) == 0x07 && ps2LastRead > 1000)  // all three buttons pressed at once
  {
    connectCloud();
    ps2LastRead = millis();
  }

  switch (ps2MouseMode)
  {
    case mmStream:  // standard mode
      if (PS2->available() >= (ps2DeviceID < 0x03 ? 3 : 4))
        ps2ProcessMouseReport();
      break;
    case mmRemote:  // on demand request mouse reports (every REPEATTIME millis)
      if (millis() - ps2LastRead >= REPEATTIME)
      {
        ps2MouseRead();
        ps2LastRead = millis();
      }
      break;
#if defined(DEBUG_PS2) || defined(DEBUG_DO_MOUSE)
    case mmReset:
      ps2MouseMode = ps2MouseInit();
      break;
    default:
      Serial.print("?? ");
      Serial.println(ps2MouseMode, HEX);
    case mmWrap:
      doDebugging();
      break;
#else
    case mmReset:
    default:
      ps2MouseMode = ps2MouseInit();
      break;
#endif
    }

#if defined(DEBUG_PS2) || defined(DEBUG_DO_MOUSE)
  if (Serial.available())
  {
    ps2MouseMode = mmWrap;
  }
#endif
}

MouseModes ps2MouseInit(MouseModes Mode
                       ,MouseSampleRates SampleRate
                       ,MouseResolutions Resolution)
{
  MouseModes mm = mmReset;

  PS2->reset();
  delay(5 * WAIT4PS2REPLY);        // wait reset to finish

  // try to set as 5 button scroll mouse (magic sample rate sequence 200, 200, 80)
  PS2->write(0xF3);                // set sample rate
  dumpACK();                       // allow time for reply (1 send + 1 receive)
  PS2->write(ms200);               // 0xC8
  dumpACK();
  PS2->write(0xF3);
  dumpACK();
  PS2->write(ms200);               // 0xC8
  dumpACK();
  PS2->write(0xF3);
  dumpACK();
  PS2->write(ms80);                // 0x50
  dumpACK();
  PS2->flush();

  PS2->write(0xF2);                // now get device ID
  delay(3 * WAIT4PS2REPLY);        // allow time for reply ( 1 send + 2 receive)
  if (PS2->read() == 0xFA)         // check ACK
    ps2DeviceID = PS2->read();

  if (ps2DeviceID != 0x04)         // try something else
  {
    PS2->flush();                  // clean slate
    // try to set as 3 button scroll mouse (magic sample rate sequence 200, 100, 80)
    PS2->write(0xF3);              // set sample rate
    dumpACK();                     // allow time for reply (1 send + 1 receive)
    PS2->write(ms200);             // 0xC8
    dumpACK();
    PS2->write(0xF3);
    dumpACK();
    PS2->write(ms100);             // 0x64
    dumpACK();
    PS2->write(0xF3);
    dumpACK();
    PS2->write(ms80);              // 0x50
    dumpACK();
    PS2->flush();                  // drop all ACKs

    PS2->write(0xF2);              // now get device ID
    delay(3 * WAIT4PS2REPLY);      // allow time for reply
    if (PS2->read() == 0xFA)       // check ACK
      ps2DeviceID = PS2->read();
  }

#if defined(DEBUG_DO_MOUSE)
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
      ps2DeviceID = mmReset;
      break;
  }
#endif

  delay(5 * WAIT4PS2REPLY);
  PS2->flush();                    // drop all ACKs

  if (SampleRate != msNone)
  {
    PS2->write(0xF3);             // set sample rate
    delay(WAIT4PS2REPLY);
    if (PS2->read() == 0xFA)
    {
      PS2->write(SampleRate);
      dumpACK();
    }
  }

  if (Resolution != mrNone)
  {
    PS2->write(0xE8);             // set resolution
    delay(WAIT4PS2REPLY);
    if (PS2->read() == 0xFA)
    {
      PS2->write(Resolution);
      dumpACK();
    }
  }

  switch (Mode)
  {
    case mmStream:
      PS2->write(0xEA);              // set stream mode (should be default anyway)
      delay(WAIT4PS2REPLY);
      if (PS2->read() == 0xFA)
      {
        mm = Mode;
        PS2->write(0xF4);            // enable reporting for stream mode
      }
      break;
    case mmRemote:
      PS2->write(0xF0);              // remote mode (mouse needs to be asked for data)
      delay(WAIT4PS2REPLY);
      if (PS2->read() == 0xFA)
      {
        mm = Mode;
      }
      break;
    case mmWrap:
      PS2->write(0xEE);             // enable  Wrap Mode (echo back each byte from host)
      delay(WAIT4PS2REPLY);
      if (PS2->read() == 0xFA)
      {
        mm = Mode;
      }
      //PS2->write(0xEC);          // disable Wrap Mode (echo back each byte from host)
      break;
    default:
      mm = mmReset;
      break;
  }

  delay(5 * WAIT4PS2REPLY);
  PS2->flush();

  return mm;
}

void ps2MouseRead()
{
  uint8_t  cnt     = (ps2DeviceID < 0x03 ? 3 : 4); // how many bytes to expect
  uint32_t timeout = millis();

  PS2->flush();

  // request a mouse report
  PS2->write(0xEB);          // give me data!
  delay(WAIT4PS2REPLY);
  PS2->read();               // ignore ACK
  while (PS2->available() < cnt && millis() - timeout < 10 * WAIT4PS2REPLY);
  if (PS2->available() == cnt)
    ps2ProcessMouseReport();
#if defined(DEBUG_PS2) || defined(DEBUG_DO_MOUSE)
  else
    Serial.print('.');
#endif
}

void ps2ProcessMouseReport()
{
  uint8_t  mState  = 0x00;
  int16_t  mX      = 0x00;
  int16_t  mY      = 0x00;
  int8_t   mZ      = 0x00;
  char     cB45    = ' ';  // place holder for button 4 & 5 if available

  pinSetFast(D7);

  // see http://www.computer-engineering.org/ps2mouse/
  mState = PS2->read() & 0x3F;  // don't care for carry flags
  // usually this wouldn't be necessary as most mice only report -127..+127
  // but the protocoll allows for -255..+255 plus a carry flag
  // -128/-256 is not allowed
  if (mX = PS2->read())  // if read byte != 0 expand sign otherwise -256 would happen
    mX |= (mState & 0x10 ? 0xFF00 : 0x0000);
  if (mY = PS2->read())  // if read byte != 0 expand sign otherwise -256 would happen
    mY |= (mState & 0x20 ? 0xFF00 : 0x0000);

  if (ps2DeviceID >= 0x03)
  {
    mZ = PS2->read();
    if (ps2DeviceID == 0x04)
    {
      mState |= (mZ & 0x30) << 2; // place button 4 & 5 in state byte
      cB45 = '_';                 // set placeholder for extra buttons
    }
    mZ &= 0x0F;                   // clear other info
    if (mZ & 0x08)                // (re)expand sign if required
      mZ |=  0xFFF0;
    // more info at http://www.computer-engineering.org/ps2mouse/
  }

  mouseX += (int)mX;
  mouseY += (int)mY;
  mouseZ += (int)mZ;
  sprintf(mouseB, "%c%c%c%c%c", mState & 0x40 ? '4' : cB45
                              , mState & 0x01 ? 'L' : 'l'
                              , mState & 0x04 ? 'M' : 'm'
                              , mState & 0x02 ? 'R' : 'r'
                              , mState & 0x80 ? '5' : cB45);

#if defined(DEBUG_DO_MOUSE)
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

#if defined(SPARK_USB_MOUSE)
  // with USB_HID_Mouse this could be passed on to a computer
  if (mX || mY || (mZ & 0x0F))
    Mouse.move(mX, -mY, -mZ);

  if (mState != lastMouseState)
  {
    // which standard buttons have
    uint8_t msk = (mState ^ lastMouseState) & 0x07;
    // which did just get pressed
    uint8_t prs = mState & msk;
    // which did just get released
    uint8_t rls = lastMouseState & msk;

    // report changes to USB HID
    Mouse.press(prs);
    Mouse.release(rls);
  }
#endif
  lastMouseState = mState;

  pinResetFast(D7);
}

// -------------              Spark.functions              -------------

int mouseReset(String cmd)
{
  lastMouseState =
  mouseB[0] =
  mouseX =
  mouseY =
  mouseZ = 0;

  return 0;
}

void connectCloud()
{
  if (!Spark.connected())
  {
    PS2->suspend();
    Spark.connect();
  }
  mouseReset(NULL);
  ps2MouseMode = ps2MouseInit(ps2MouseMode);
}

// ------------- suplemental functions e.g. for debugging -------------

inline void dumpACK()
{
  delay(WAIT4PS2REPLY);

#if defined(DEBUG_PS2)
  uint8_t r;

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

#if defined(DEBUG_PS2) || defined(DEBUG_DO_MOUSE)
void doDebugging()
{
  static char echo[8] = {'\0'};
  static int  echoLen = 0;
  static long echoByte = 0;

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

    // Spark.connect()
    if (echo[0] == 'S' || echo[0] == 's')
    {
      connectCloud();
      return;
    }

    // Beware! You need to flash firmware via USB (dfu-util) after this
    if (echo[0] == 'X')
    {                        // enter dfu
      Serial.println("Disconnect serial monitor - prepare for DFU");
      delay(5000);           // give us some time to close serial monitor
      FLASH_OTA_Update_SysFlag = 0x0000;
      Save_SystemFlags();
      BKP_WriteBackupRegister(BKP_DR10, 0x0000);
      USB_Cable_Config(DISABLE);
      NVIC_SystemReset();
    }

    echoByte = strtol(echo, NULL, 16);
    if (echoByte > 0 && ps2MouseMode != mmWrap)
    {
      PS2->write(mmWrap);    // enable  Wrap Mode (echo back each byte from host)
      //PS2->write(0xEC);    // disable Wrap Mode (echo back each byte from host)
      delay(WAIT4PS2REPLY);
      ps2MouseMode = mmWrap;
    }
    else if (echoByte == mmReset)
    {
      ps2MouseMode = mmReset;
      echoByte = 0;
      return;
    }
    else if (echoByte == mmStream)  // enable Stream reporting
    {
      ps2MouseMode = mmStream;
      PS2->write(mmStream);
      delay(WAIT4PS2REPLY);
      echoByte = 0xF4; // enable reporting
    }

    if (echoByte > 0)
    {
      PS2->write((byte)echoByte);
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
#endif
