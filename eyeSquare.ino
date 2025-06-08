
#define SOFTI2C

#ifdef SOFTI2C
constexpr byte PIN_SCL = 16;
constexpr byte PIN_SDA = 2;


// https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf -- see p64 for startup flow

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
// 

namespace oled
{
  constexpr byte Addr = 0x3c;   // i2c address of screen driver
  
  constexpr byte CmdSetMemAddrMode = 0x20;
  constexpr byte MemAddrMode_Horiz = 0;
  constexpr byte MemAddrMode_Vert = 1;
  constexpr byte MemAddrMode_Page = 2;

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
    CmdSetComScanDir_Flip,
    CmdSetComPinCfg, 2,

    CmdSetMemAddrMode, MemAddrMode_Horiz,

    CmdSetContrast, 0x7f,

    CmdDisplayOffEnable,
    CmdSetColors_Std,

    CmdSetDispOscFreq, 0x80,
    CmdSetChgPumpCfg, 0x14,

    CmdOn,
  };

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


uint16_t pat = 0xf090;//0xac65;

bool oledPattern()
{
  if (!i2cStart(oled::Addr))
  {
    Serial.println("oledPattern: i2cStart failed");
    return false;
  }

  // D header byte
  if (!i2cSendByte(0x40))
  {
    Serial.println("oledPattern: i2cWriteByte dummy failed");
    return false;
  }

  for (int i=0; i<(128*64/8); ++i)
  {
    byte pixels = (i < 32) ? 0x00 : 0xff;
    if (!i2cSendByte(byte(pat)))
    {
      Serial.println("oledPattern: i2cWriteByte pat failed");
      return false;
    }

    pat = (pat << 1) ^ (pat >> 15);
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

void loop()
{
  delay(100);

  digitalWrite(0, LOW);

  Serial.println("oledOn...");
  oledStartup();
  delay(500);

  Serial.println("send pattern...");
  digitalWrite(0, HIGH);
  oledPattern();
  delay(2000);
  
  digitalWrite(0, LOW);
  Serial.println("oledOff...");
  oledOff();
  
  digitalWrite(0, HIGH);
  delay(500);
}