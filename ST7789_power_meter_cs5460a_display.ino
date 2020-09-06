// CS5460A-based plug power meter color display
// Required RREFont and DigiFont
// (c) 2020 Pawel A. Hernik
// Based on Karl Hagstrom's and Jens Jensen's reverse engineering
// YouTube video: https://youtu.be/XAVUyOCNxC0
 
// *********  WARNING / DANGER *********
// Ground reference of PCB inside meter is tied to HOT (Line). It is at mains level. 
// Use galvanic isolation, e.g. optocouplers, etc if you want to wire this up to something else.
// Be sure you know what your are doing around lethal mains-level voltages. Use at your own risk!
// *********  WARNING / DANGER *********


/*
 ST7789 240x240 IPS (without CS pin) connections (only 6 wires required):

 #01 GND -> GND
 #02 VCC -> VCC (3.3V only!)
 #03 SCL -> D13/PA5/SCK
 #04 SDA -> D11/PA7/MOSI
 #05 RES -> D9 /PA0 or any digital
 #06 DC  -> D10/PA1 or any digital
 #07 BLK -> NC
*/

#include <SPI.h>
#include <Adafruit_GFX.h>
#define TFT_DC    10
#define TFT_RST   9
#define BUTTON    3
#define DHT11_PIN 7
#include <Arduino_ST7789_Fast.h>

#define SCR_WD 240
#define SCR_HT 240
Arduino_ST7789 lcd = Arduino_ST7789(TFT_DC, TFT_RST);

// use only ENABLE_RRE_16B = 1 to save 4KB of flash
#include "RREFont.h"
#include "rre_term_10x16.h"
RREFont font;
// needed for RREFont library initialization, define your fillRect
void customRectRRE(int x, int y, int w, int h, int c) { return lcd.fillRect(x, y, w, h, c); }  

// -----------------
#include "DigiFont.h"
// needed for DigiFont library initialization, define your customLineH, customLineV and customRect
void customLineH(int x0,int x1, int y, int c) { lcd.drawFastHLine(x0,y,x1-x0+1,c); }
void customLineV(int x, int y0,int y1, int c) { lcd.drawFastVLine(x,y0,y1-y0+1,c); } 
void customRect(int x, int y,int w,int h, int c) { lcd.fillRect(x,y,w,h,c); } 
DigiFont digi(customLineH,customLineV,customRect);
// -----------------

// enable or disable debug fields (Vcc and SPI packets counter)
#define DEBUG(x) x
//#define DEBUG(x)

// Sniffing the SPI CLK and MISO from the chip to read voltage, current, power, etc.
// CS5460A datasheet: http://www.cirrus.com/en/pubs/proDatasheet/CS5460A_F5.pdf

#define CLKPIN  2
#define SDOPIN  3     // should be on PORTD

#define VOLTAGE_RANGE    0.250   // full scale V channel voltage
#define CURRENT_RANGE    0.050   // full scale I channel voltage (PGA 50x instead of 10x)
#define VOLTAGE_DIVIDER  1.00047*450220.0/220.0    // input voltage channel divider (R1+R2/R2), 1.00047 my multiplier
#define CURRENT_SHUNT    1.0071*620     // empirically obtained multiplier to scale Vshunt drop to I, 1.0071 my multiplier
#define FLOAT24          16777216.0  // 2^24 (converts to float24)

#define POWER_MULT       1 / 512.0
#define VOLTAGE_MULT     (float)  ( VOLTAGE_RANGE * VOLTAGE_DIVIDER / FLOAT24 )
#define CURRENT_MULT     (float)  ( CURRENT_RANGE * CURRENT_SHUNT / FLOAT24 )

float voltage = 0;
float current = 0;
float truePower = 0;
float powerFactor = 0;
float minPower = 999;
float maxPower = 0;
float energy = 0;
unsigned long startTime = 0;
unsigned long minPowerTime = 0;
unsigned long maxPowerTime = 0;
unsigned long lastPowerTime = 0;

////////////////////////////////////////////
volatile uint8_t clkHighCnt = 0;  // number of CLK-highs (find start of a byte)
volatile boolean inSync = false;  // as long as we are in SPI-sync
volatile boolean byteReady = false;
volatile uint8_t spiByte = 0;
volatile uint8_t tmpByte = 0;
volatile uint8_t bitCnt = 0;

void clkInt()
{
  //if we are trying to find the sync-time (CLK goes high for 1-2ms)
  if(inSync==false) {
    clkHighCnt = 0;
    //Register how long the ClkHigh is high to evaluate if we are at the part wher clk goes high for 1-2 ms
    while(digitalRead(CLKPIN)==HIGH && clkHighCnt<200) {
      clkHighCnt++;
      delayMicroseconds(30);
    }
    //if the Clk was high between 1 and 2 ms than, its a start of a SPI-transmission
    if(clkHighCnt>=33 && clkHighCnt<=67) {
       bitCnt = 0;
       tmpByte = 0;
       inSync = true;
    }
  } else {
    tmpByte |= bitRead(PIND, SDOPIN) << (7 - bitCnt);
    if(++bitCnt>7) {
       bitCnt = 0;
       spiByte = tmpByte;
       byteReady = true;
       tmpByte = 0;
    }
  }
}

// --------------------------------------------------------
#define RGBIto565(r,g,b,i) ((((((r)*(i))/255) & 0xF8) << 8) | ((((g)*(i)/255) & 0xFC) << 3) | ((((b)*(i)/255) & 0xFC) >> 3)) 

const uint16_t v1Col1 = RGBIto565(100,250,100,255);
const uint16_t v1Col2 = RGBIto565(100,250,100,200);
const uint16_t v1Col0 = RGBIto565(100,250,100,40);
const uint16_t v2Col1 = RGBIto565(255,120,120,255);
const uint16_t v2Col2 = RGBIto565(255,120,120,200);
const uint16_t v2Col0 = RGBIto565(255,120,120,40);
const uint16_t v3Col1 = RGBIto565(120,250,250,255);
const uint16_t v3Col2 = RGBIto565(120,250,250,200);
const uint16_t v3Col0 = RGBIto565(120,255,250,40);
const uint16_t v4Col1 = RGBIto565(255,250,100,255);
const uint16_t v4Col2 = RGBIto565(255,250,100,200);
const uint16_t v4Col0 = RGBIto565(255,250,100,40);
const uint16_t v5Col1 = RGBIto565(50,50,250,255);
const uint16_t v5Col2 = RGBIto565(50,50,250,200);
const uint16_t v5Col0 = RGBIto565(50,50,250,50);

const uint16_t vdCol1 = RGBIto565(255,255,255,160);
const uint16_t vdCol0 = RGBIto565(255,255,255,30);

char txt[20];
const int8_t dw=34,dh=61,sp=4,th=9;
int8_t mode=0,lastMode=-1;
int8_t cMode=0,cModeLast=0;
int8_t pMode=0,pModeLast=0;
int8_t pminMode=0,pminModeLast=0;
int8_t pmaxMode=0,pmaxModeLast=0;
unsigned int cnt=0;

long readVcc() 
{
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  //int wait=1000; while(bit_is_set(ADCSRA,ADSC) && --wait>0);
  while(bit_is_set(ADCSRA,ADSC));
  long result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  if(result<0) result=0;
  if(result>6000) result=6000;
  return result;
}

int showVal(float v, int x, int y, int w, int d, uint16_t col1, uint16_t col2, uint16_t col0)
{
  dtostrf(v,w,d,txt);
  digi.setColors(col1,col2,col0);
  return digi.printNumber7(txt,x,y);
}

void showConst()
{
  lcd.fillScreen(BLACK);
  font.setScale(2);
  if(mode==0) {
    font.setColor(v3Col1); font.printStr(ALIGN_RIGHT,dh*0+2,"W");
    font.setColor(v1Col1); font.printStr(ALIGN_RIGHT,dh*1+2,"V");
    font.setColor(v2Col1); font.printStr(ALIGN_RIGHT,dh*2+2,"A");
    font.setColor(v4Col1); font.printStr((dw+4)*3+th+16,dh*3+2,"PF");
  } else {
    font.setColor(v4Col1); font.printStr(ALIGN_RIGHT,dh*0+2,"Wh");
    font.setColor(v3Col1); font.printStr(ALIGN_RIGHT,dh*1+2,"W");
    font.setColor(v5Col1); font.printStr((28+4)*5+16,dh*2+2,"W");
    font.setColor(v2Col1); font.printStr((28+4)*5+16,dh*3+2,"W");
  }
}

void showTime(uint32_t t, int x,int y, uint16_t col1, uint16_t col0)
{
  // 2 digit display [99]
  // 0 - 90s    -> seconds
  // 90s - 90m  -> minutes
  // 90m - 90h  -> hours
  // 90h+       -> days
  char unit[2] = "s";
  if(t>90) { t/=60; unit[0]='m'; }
  if(t>90) { t/=60; unit[0]='h'; }
  if(t>90) { t/=24; unit[0]='d'; }
  showVal(t, x,y, 2,0, col1,col1,col0);
  font.setScale(1);
  font.setColor(col1,BLACK);
  //Serial.println(font.strWidth("h"));  Serial.println(font.strWidth("m"));  Serial.println(font.strWidth("s"));  Serial.println(font.strWidth("d"));
  font.printStr(ALIGN_RIGHT,y,unit);
}

void showEnergy()
{
  // 3.1 digit display [9.999]
  // 0 - 0.999Wh -> mWh
  // 1 - 999Wh   -> Wh
  // 1000Wh+     -> kWh
  float en = energy;
  char unit[2]=" ";
  if(en<1.0)    { en*=1000.0; unit[0]='m'; } else
  if(en>1000.0) { en/=1000.0; unit[0]='k'; }
  showVal(en, 0,dh*0, 5,1, v4Col1,v4Col2,v4Col0);
  font.setScale(2);
  font.setColor(v4Col1);
  int wd=font.strWidth("mWh");
  //Serial.println(font.strWidth("m"));  Serial.println(font.strWidth("k"));  Serial.println(font.strWidth(" "));
  lcd.fillRect(SCR_WD-wd,2,9*2,16*2,BLACK);
  font.printStr(SCR_WD-wd,dh*0+2,unit);
}

void showCurrent(int y)
{
  // 1.4 [9.9999] (<1A) or 2.3 [99.999] (>1A) digit display
  cMode = (current>=10) ? 1 : 0;
  if(cMode!=cModeLast) lcd.fillRect(dw+sp,y,dw+sp+th,dh-5,BLACK);
  cModeLast = cMode;
  showVal(current, 0,y, 6,cMode?3:4, v2Col1,v2Col2,v2Col0);
}

void showPower(int y)
{
  // 3.2 [999.99] (<1000W) or 4.1 [9999.9] (>1000W) digit display
  pMode = (truePower>=1000) ? 1 : 0;
  if(pMode!=pModeLast) lcd.fillRect((dw+sp)*3,y,dw+sp+th,dh-5,BLACK);
  pModeLast = pMode;
  showVal(truePower, 0,y, 6,pMode?1:2, v3Col1,v3Col2,v3Col0);
}

void showMinMax(int y)
{
  // 3.2 [999.99] (<1000W) or 4.1 [9999.9] (>1000W) digit display
  digi.setSize7(28,61-5,th,th/2); digi.setSpacing(4);
  pminMode = (minPower>=1000) ? 1 : 0;
  pmaxMode = (maxPower>=1000) ? 1 : 0;
  if(pminMode!=pminModeLast) lcd.fillRect((28+4)*3,y   ,28+4+th,dh-5,BLACK);
  if(pmaxMode!=pmaxModeLast) lcd.fillRect((28+4)*3,y+dh,28+4+th,dh-5,BLACK);
  pmaxModeLast = pmaxMode;
  pminModeLast = pminMode;
  showVal(minPower, 0,y   , 6,pminMode?1:2, v5Col1,v5Col2,v5Col0);
  showVal(maxPower, 0,y+dh, 6,pmaxMode?1:2, v2Col1,v2Col2,v2Col0);

  digi.setSize7(15-2,28,3,0); digi.setSpacing(2);
  showTime((millis()-minPowerTime)/1000, SCR_WD-2*15-11,y   +56-28,v5Col1,v5Col0);
  showTime((millis()-maxPowerTime)/1000, SCR_WD-2*15-11,y+dh+56-28,v2Col1,v2Col0);
}

void showData()
{
  if(truePower<9999 && truePower>0) {
    if(millis()-startTime>5000) {
      if(truePower>maxPower) { maxPower=truePower; maxPowerTime = millis(); }
      if(truePower<minPower) { minPower=truePower; minPowerTime = millis(); }
    } else  maxPower = minPower = truePower;
    float perSecond = (millis()-lastPowerTime)/1000.0;
    energy += truePower*perSecond/3600.0; // per hour
    lastPowerTime = millis();
  }

  digi.setSize7(dw,dh-5,th,th/2);
  digi.setSpacing(sp);
  mode = (cnt&15)<6 ? 0 : 1;
//mode=1;
  if(mode!=lastMode) showConst();
  lastMode = mode;
  if(mode==0) {
    showPower(dh*0);
    showVal(voltage,     0,dh*1, 6,2, v1Col1,v1Col2,v1Col0);
    showCurrent(dh*2);
    showVal(powerFactor, 0,dh*3, 3,2, v4Col1,v4Col2,v4Col0);
    DEBUG(digi.setSize7(12-2,22,3,0); digi.setSpacing(2));
    DEBUG(showVal(readVcc()/1000.0, SCR_WD-3*12-4,SCR_HT-22, 3,2, vdCol1,vdCol1,vdCol0));
    DEBUG(showVal(cnt, SCR_WD-3*12,dh*3, 3,0, vdCol1,vdCol1,vdCol0));
  } else {
    showEnergy();
    showPower(dh*1);
    showMinMax(dh*2);

    DEBUG(digi.setSize7(12-2,22,3,0); digi.setSpacing(2));
    DEBUG(showVal(cnt, SCR_WD-3*12,dh*3, 3,0, vdCol1,vdCol1,vdCol0));
  }
  if(++cnt>999) cnt=0;
}


void setup() 
{
  Serial.begin(9600);
  pinMode(SDOPIN, INPUT);
  pinMode(CLKPIN, INPUT_PULLUP);
  lcd.init(SCR_WD, SCR_HT);
  font.init(customRectRRE, SCR_WD, SCR_HT); // custom fillRect function and screen width and height values 
  font.setFont(&rre_term_10x16);
  lastPowerTime = startTime = millis();
  showData();
  //delay(100);
  attachInterrupt(0, clkInt, RISING);
}

uint32_t readSPI(int n)
{
  uint32_t spi32 = 0;
  for(int i=0;i<n;i++) {
    byteReady = false;
    while(!byteReady);
    spi32 = (spi32<<8) | spiByte;
  }
  return spi32;
}


void readPowerMeter(void)
{
  if(!inSync) return;
  uint32_t raw = 0;
  readSPI(4);
  raw = readSPI(4); // voltage or status register
  if((raw & 0xffff) == 0x03c1) return;  // works
  //if((raw&0xffffff) == 0x009003C1) return; // check if status register has conversion ready ( DRDY=1, ID=15 )

  voltage = raw * VOLTAGE_MULT;
  if(voltage<0) voltage=0;
  if(voltage>999) voltage=999;

  raw = readSPI(4);
  current = raw * CURRENT_MULT;
  //if(current<0.0001) current=0;
  if(current>99) current=99;

  raw = readSPI(4);
  truePower = raw * POWER_MULT;
  //if(truePower<0.2) truePower=0;
  if(truePower>9999) truePower=9999;

  float apparentPower = voltage * current;
  powerFactor = apparentPower>0 ? truePower / apparentPower : 0;
  if(powerFactor<0) powerFactor=0;
  if(powerFactor>1) powerFactor=1;

  showData();
  inSync = false;
}

void loop()
{
  readPowerMeter();
  /*
  // display tests
  voltage = random(22000,25000)/100.0;
  truePower = random(1,630000)/100.0;
  current = random(1,90000)/1000.0;
  //truePower = 4600;
  //Serial.println(truePower);
  showData(); delay(800);
  */
 }

