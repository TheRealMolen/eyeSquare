
#define SOFTI2Cx

#ifdef SOFTI2C
constexpr byte PIN_SCL = 16;
constexpr byte PIN_SDA = 2;

constexpr int i2cMaxChunkLength = 16384;

inline void i2cTick()
{
  delayMicroseconds(4); // 400kbits ish
}
inline void i2cHalfTickA()
{
  delayMicroseconds(2); // 400kbits ish
}
inline void i2cHalfTickB()
{
  delayMicroseconds(2); // 400kbits ish
}

inline void i2cIdle()
{
  digitalWrite(PIN_SCL, HIGH);
  digitalWrite(PIN_SDA, HIGH);
  i2cTick();
}

inline void i2cClockLo()
{
  digitalWrite(PIN_SCL, LOW);
  i2cHalfTickA();
}

inline void i2cClockHi()
{
  i2cHalfTickB();
  digitalWrite(PIN_SCL, HIGH);
}

bool i2cAckOk()
{
  pinMode(PIN_SDA, INPUT);
  i2cClockHi();
  int ackBit = digitalRead(PIN_SDA);
  i2cClockLo();
  pinMode(PIN_SDA, OUTPUT);

  return (ackBit == 0);
}

bool i2cStart(byte address)
{
  pinMode(PIN_SDA, OUTPUT);

  // START
  digitalWrite(PIN_SDA, LOW);
  i2cTick();

  i2cClockLo();

  // ADDRESS
  byte mask = (1 << 6);
  while (mask)
  {
    digitalWrite(PIN_SDA, (address & mask) != 0);
    i2cClockHi();
    i2cClockLo();

    mask = mask >> 1;
  }

  // R/W#
  digitalWrite(PIN_SDA, 0);   // always WRITE
  i2cClockHi();
  i2cClockLo();

  // ACK
  bool ackOk = i2cAckOk();
  return ackOk;
}

void i2cStop()
{
  i2cClockHi();
  digitalWrite(PIN_SDA, HIGH);
  i2cTick();
}

bool i2cSendByte(byte b)
{
  byte mask = (1 << 7);
  while (mask)
  {
    digitalWrite(PIN_SDA, (b & mask) != 0);
    i2cClockHi();
    i2cClockLo();

    mask = mask >> 1;
  }

  return i2cAckOk();
}

void i2cInit()
{
  pinMode(PIN_SCL, OUTPUT);
  pinMode(PIN_SDA, OUTPUT);

  i2cIdle();
}

#else // using hardware i2c

#include <Wire.h>

#ifdef I2C_BUFFER_LENGTH
constexpr int i2cMaxChunkLength = I2C_BUFFER_LENGTH;
#elif BUFFER_LENGTH // :|
constexpr int i2cMaxChunkLength = BUFFER_LENGTH;
#else
constexpr int i2cMaxChunkLength = 32;
#endif

void i2cInit()
{
  Wire.begin();
}

bool i2cStart(byte address)
{
  Wire.beginTransmission(address);
  return true;
}

void i2cStop()
{
  int err = Wire.endTransmission(1);
  if (err)
  {
    Serial.print("hw_i2c stop failed. error:");
    Serial.println(err);
  }
}

bool i2cSendByte(byte b)
{
  Wire.write(b);
  return true;
}

#endif


//--------------------------------------------------------------------------------
// OLED - driver for adafruit 128x32 monochrome OLED over I2C
// https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf -- see p64 for startup flow
// 

namespace oled
{
  constexpr byte Addr = 0x3c;   // i2c address of screen driver
  
  constexpr byte CmdSetMemAddrMode = 0x20;
  constexpr byte MemAddrMode_Horiz = 0;
  constexpr byte MemAddrMode_Vert = 1;
  constexpr byte MemAddrMode_Page = 2;

  constexpr byte CmdSetColumnRange = 0x21;
  constexpr byte CmdSetPageRange_NonPaged = 0x22;

  constexpr byte CmdDeactivateScroll = 0x2e;
  constexpr byte CmdActivateScroll = 0x2f;

  constexpr byte CmdSetDispStartLine = 0x40;

  constexpr byte CmdSetContrast = 0x81;
  constexpr byte CmdSetChgPumpCfg = 0x8D;

  constexpr byte CmdSetSegRemap_Std = 0xA0;
  constexpr byte CmdSetSegRemap_Flip = 0xA1;
  constexpr byte CmdDisplayOffEnable = 0xA4;
  constexpr byte CmdDisplayOnEnable = 0xA5;
  constexpr byte CmdSetColors_Std = 0xA6;
  constexpr byte CmdSetColors_Inv = 0xA7;
  constexpr byte CmdSetMuxRatio = 0xA8;
  constexpr byte CmdOff = 0xAE;
  constexpr byte CmdOn = 0xAF;
  
  constexpr byte CmdSetComScanDir_Std = 0xC0;
  constexpr byte CmdSetComScanDir_Flip = 0xC8;

  constexpr byte CmdSetDispOffset = 0xD3;
  constexpr byte CmdSetDispOscFreq = 0xD5;
  constexpr byte CmdSetPrecharge = 0xD9;
  constexpr byte CmdSetComPinCfg = 0xDA;

  constexpr byte CfgDispWidth = 128;
  constexpr byte CfgDispHeight = 32;


  constexpr byte StartupList[] =
  {
    CmdOff,

    CmdSetMuxRatio, CfgDispHeight-1,
    CmdSetDispOffset, 0,
    CmdSetDispStartLine,
    CmdSetSegRemap_Std,
    CmdSetComScanDir_Std,
    CmdSetComPinCfg, 2,

    CmdSetMemAddrMode, MemAddrMode_Horiz,

    CmdSetContrast, 0x8f,

    CmdDisplayOffEnable,
    CmdSetColors_Std,

    CmdSetPrecharge, 0x22,

    CmdDeactivateScroll,

    CmdSetDispOscFreq, 0x80,
    CmdSetChgPumpCfg, 0x14,

    CmdOn,
  };
  

  byte backBuf[CfgDispHeight * CfgDispWidth / 8];

}



bool oledCmd(byte cmd)
{
  // header - Co=0, D/C#=0, six zeros
  if (!i2cSendByte(byte(0)))
    return false;

  return i2cSendByte(cmd);
}

bool oledSendMultiCmd(const byte* cmds, byte nCmds)
{
  // header - Co=0, D/C#=0, six zeros
  if (!i2cSendByte(byte(0)))
    return false;
  
  for (byte i=0; i<nCmds; ++i)
  {
    const bool success = i2cSendByte(cmds[i]);
    if (!success)
    {
      Serial.print("i2cSendMultiCmd: failed on cmd ");
      Serial.println(int(i));
      return false;
    }
  }

  return true;
}

bool oledOn()
{
  if (!i2cStart(oled::Addr))
  {
    Serial.println("oledOn: i2cStart failed");
    return false;
  }
  if (!oledCmd(oled::CmdOn))
  {
    Serial.println("oledOn: i2cCmd(ON) failed");
    return false;
  }
  i2cStop();

  return true;
}

void oledClearBuf(byte val)
{
  memset(oled::backBuf, val, sizeof(oled::backBuf));
}


bool oledStartup()
{
  if (!i2cStart(oled::Addr))
  {
    Serial.println("oledStartup: i2cStart failed");
    return false;
  }
  
  if (!oledSendMultiCmd(oled::StartupList, sizeof(oled::StartupList)))
  {
    return false;
  }

  i2cStop();

  oledClearBuf(0);

  return true;
}


bool oledDisplayOn()
{
  if (!i2cStart(oled::Addr))
  {
    Serial.println("oledDisplayOn: i2cStart failed");
    return false;
  }
  if (!oledCmd(oled::CmdDisplayOnEnable))
  {
    Serial.println("oledDisplayOn: i2cCmd(ON) failed");
    return false;
  }
  i2cStop();

  return true;
}

bool oledOff()
{
  if (!i2cStart(oled::Addr))
  {
    Serial.println("oledOff: i2cStart failed");
    return false;
  }
  if (!oledCmd(oled::CmdOff))
  {
    Serial.println("oledOff: i2cCmd(OFF) failed");
    return false;
  }
  i2cStop();

  return true;
}

void oledFlip()
{
  static const byte setupBuf[] =
  {
    oled::CmdSetPageRange_NonPaged, 0, 0x3f,
    oled::CmdSetColumnRange, 0, oled::CfgDispWidth - 1,
  };
  i2cStart(oled::Addr);
  oledSendMultiCmd(setupBuf, sizeof(setupBuf));
  i2cStop();

  i2cStart(oled::Addr);
  i2cSendByte(0x40); // D header byte

  const byte* bufEnd = oled::backBuf + sizeof(oled::backBuf);

  int chunksize = 1;
  for (const byte* bufCurr = oled::backBuf; bufCurr != bufEnd; ++bufCurr)
  {
    i2cSendByte(*bufCurr);

    ++chunksize;
     if (chunksize == i2cMaxChunkLength)
    {
      i2cStop();
      i2cStart(oled::Addr);
      i2cSendByte(0x40); // D header byte
      chunksize = 1;
    }
  }

  i2cStop();
}


uint16_t pat = 0xf090;//0xac65;

bool oledPattern()
{
  static const byte setupBuf[] =
  {
    oled::CmdSetPageRange_NonPaged, 0, 0x3f,
    oled::CmdSetColumnRange, 0, oled::CfgDispWidth - 1,
  };
  i2cStart(oled::Addr);
  oledSendMultiCmd(setupBuf, sizeof(setupBuf));
  i2cStop();

  i2cStart(oled::Addr);
  i2cSendByte(0x40); // D header byte

  int chunksize = 1;
  for (int y=0; y < oled::CfgDispWidth; ++y)
  {
    for (int x=0; x < oled::CfgDispHeight; x+=8)
    {
      int maxX = x+7;
      
      constexpr int r = 30;
      if ((x * x) + (y * y) < (r*r))
        i2cSendByte(0xff);
      else if ((maxX * maxX) + (y * y) > (r*r))
        i2cSendByte(0x00);
      else
        i2cSendByte(0xd4);

      ++chunksize;
      if (chunksize == i2cMaxChunkLength)
      {
        i2cStop();
        i2cStart(oled::Addr);
        i2cSendByte(0x40); // D header byte
        chunksize = 1;
      }
    }
  }

  i2cStop();

  return true;
}


void setup()
{
  Serial.begin(115200);
  i2cInit();
  delay(100);

  pinMode(0, OUTPUT);
}

const byte Font5x8_AZ[] =
{
  0xfc, 0x12, 0x11, 0x12, 0xfc,   // A
  0xff, 0x91, 0x91, 0x92, 0x6c,   // B
  0x7e, 0x81, 0x81, 0x82, 0x44,   // C
  0xff, 0x81, 0x81, 0x82, 0x7c,   // D
  0xff, 0x91, 0x91, 0x81, 0x80,   // E
  0xff, 0x11, 0x11, 0x01, 0x00,   // F
  0x7e, 0x81, 0x81, 0x92, 0x74,   // G
  0xff, 0x20, 0x20, 0x20, 0xfe,   // H
  0x00, 0x81, 0xff, 0x81, 0x00,   // I
  0x20, 0x41, 0x81, 0x81, 0x7f,   // J
  0xff, 0x10, 0x24, 0x42, 0x80,   // K
  0xff, 0x80, 0x80, 0x80, 0x00,   // L
  0xff, 0x02, 0x0c, 0x02, 0xff,   // M
  0xff, 0x08, 0x01, 0x02, 0xfe,   // N
  0x7e, 0x81, 0x81, 0x82, 0x7c,   // O
  0xff, 0x21, 0x21, 0x22, 0x1c,   // P
  0xff, 0x81, 0xa1, 0x42, 0xbc,   // Q
  0xff, 0x21, 0x21, 0x62, 0x9c,   // R
  0x4e, 0x91, 0x91, 0x92, 0x64,   // S
  0x01, 0x01, 0xff, 0x01, 0x01,   // T
  0x7f, 0x80, 0x80, 0x80, 0xfe,   // U
  0x3f, 0x40, 0x80, 0x40, 0x3e,   // V
  0x7f, 0x80, 0x60, 0x80, 0x7e,   // W
  0xc7, 0x28, 0x10, 0x28, 0xc6,   // X
  0x4f, 0x90, 0x90, 0x90, 0x7e,   // Y
  0xc1, 0xa1, 0x91, 0x89, 0x87,   // Z
};


const byte Font5x7_AZaz09[] = {
  0x78, 0x24, 0x22, 0x24, 0x78,   // A
  0x7e, 0x4a, 0x4a, 0x4a, 0x34,   // B
  0x3c, 0x42, 0x42, 0x42, 0x44,   // C
  0x7e, 0x42, 0x42, 0x44, 0x38,   // D
  0x7e, 0x4a, 0x4a, 0x42, 0x00,   // E
  0x7e, 0x0a, 0x0a, 0x02, 0x00,   // F
  0x3c, 0x42, 0x42, 0x52, 0x74,   // G
  0x7e, 0x20, 0x20, 0x20, 0x7c,   // H
  0x00, 0x42, 0x7e, 0x42, 0x00,   // I
  0x20, 0x42, 0x42, 0x42, 0x3e,   // J
  0x7e, 0x10, 0x10, 0x28, 0x44,   // K
  0x7e, 0x40, 0x40, 0x40, 0x00,   // L
  0x7e, 0x04, 0x18, 0x04, 0x7e,   // M
  0x7e, 0x08, 0x10, 0x20, 0x7c,   // N
  0xbc, 0x42, 0x42, 0x44, 0x38,   // O
  0x7e, 0x0a, 0x0a, 0x0a, 0x0c,   // P
  0x3c, 0x42, 0x52, 0x24, 0x58,   // Q
  0x7e, 0x0a, 0x0a, 0x1a, 0x6c,   // R
  0x44, 0x4a, 0x4a, 0x4a, 0x30,   // S
  0x02, 0x02, 0x7e, 0x02, 0x02,   // T
  0x3e, 0x40, 0x40, 0x40, 0x3c,   // U
  0x1e, 0x20, 0x40, 0x20, 0x1c,   // V
  0x3e, 0x20, 0x40, 0x20, 0x1c,   // W
  0x42, 0x24, 0x18, 0x24, 0x42,   // X
  0x06, 0x08, 0x78, 0x08, 0x06,   // Y
  0x42, 0x62, 0x5a, 0x46, 0x42,   // Z

  0x38, 0x44, 0x44, 0x04, 0x7c,   // a
  0x7e, 0x44, 0x44, 0x44, 0x38,   // b
  0x38, 0x44, 0x44, 0x44, 0x48,   // c
  0x38, 0x44, 0x44, 0x04, 0x7e,   // d
  0x38, 0x44, 0x54, 0x54, 0x48,   // e
  0x08, 0x7c, 0x0a, 0x02, 0x00,   // f
  0x98, 0xa4, 0xa4, 0x84, 0x78,   // g
  0x7e, 0x04, 0x04, 0x04, 0x78,   // h
  0x00, 0x3a, 0x40, 0x00, 0x00,   // i
  0x40, 0x80, 0x7a, 0x00, 0x00,   // j
  0x7e, 0x10, 0x28, 0x40, 0x00,   // k
  0x00, 0x3e, 0x40, 0x40, 0x00,   // l
  0x7c, 0x04, 0x18, 0x04, 0x78,   // m
  0x7c, 0x04, 0x04, 0x04, 0x78,   // n
  0x38, 0x44, 0x44, 0x44, 0x38,   // o
  0xfc, 0x04, 0x44, 0x44, 0x38,   // p
  0x38, 0x44, 0x44, 0x04, 0xfc,   // q
  0x7c, 0x04, 0x04, 0x04, 0x08,   // r
  0x48, 0x54, 0x54, 0x54, 0x20,   // s
  0x08, 0x3e, 0x48, 0x40, 0x00,   // t
  0x3c, 0x40, 0x40, 0x40, 0x7c,   // u
  0x1c, 0x20, 0x40, 0x20, 0x1c,   // v
  0x3c, 0x40, 0x30, 0x40, 0x3c,   // w
  0x44, 0x28, 0x10, 0x28, 0x44,   // x
  0x1c, 0xa0, 0xa0, 0xa0, 0x7c,   // y
  0x44, 0x64, 0x54, 0x4c, 0x44,   // z

  0x3c, 0x62, 0x5a, 0x46, 0x3c,   // 0
  0x08, 0x04, 0x7e, 0x00, 0x00,   // 1
  0x44, 0x62, 0x52, 0x4a, 0x44,   // 2
  0x42, 0x42, 0x4a, 0x46, 0x32,   // 3
  0x0e, 0x08, 0x08, 0x7e, 0x08,   // 4
  0x4e, 0x4a, 0x4a, 0x4a, 0x32,   // 5
  0x3c, 0x4a, 0x4a, 0x4a, 0x30,   // 6
  0x02, 0x72, 0x0a, 0x06, 0x02,   // 7
  0x34, 0x4a, 0x4a, 0x4a, 0x34,   // 8
  0x0c, 0x52, 0x52, 0x52, 0x3c,   // 9
};


void loop()
{
  delay(100);

  digitalWrite(0, LOW);

  Serial.println("oledOn...");
  oledStartup();
  delay(300);

  //Serial.println("send pattern...");
  //digitalWrite(0, HIGH);
  //oledPattern();
  //Serial.println("sent...");
  //delay(2000);

  Serial.println("clear to 0...");
  oledClearBuf(0);
  oledFlip();
  delay(100);
  
  Serial.println("clear to 11...");
  oledClearBuf(0x11);
  oledFlip();
  delay(100);
  
  Serial.println("clear to aa...");
  oledClearBuf(0xaa);
  oledFlip();
  delay(100);
  
  Serial.println("clear to f7...");
  oledClearBuf(0xf7);
  oledFlip();
  delay(100);
  
  Serial.println("clear to ff...");
  oledClearBuf(0xf7);
  oledFlip();
  delay(100);
  
  Serial.println("A...");
  oledClearBuf(0);

  byte *bufCur = oled::backBuf;
  const byte* bufEnd = oled::backBuf + sizeof(oled::backBuf);
  const byte* ltrCur = Font5x7_AZaz09;
  const byte* ltrEnd = Font5x7_AZaz09 + sizeof(Font5x7_AZaz09);

  while (bufCur != bufEnd && ltrCur != ltrEnd)
  {
    for (int i=0; i<5; ++i, ++ltrCur, ++bufCur)
      *bufCur = *ltrCur;

    ++bufCur;
  }

  oledFlip();
  delay(5000);

  digitalWrite(0, LOW);
  Serial.println("oledOff...");
  oledOff();
  
  digitalWrite(0, HIGH);
  delay(300);
}