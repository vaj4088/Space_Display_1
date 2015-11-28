// Graphics library by ladyada/adafruit with init code from Rossum 
// MIT license

#include "GL_ST7735.h"
#include "glcdfont.c"
#include <avr/pgmspace.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include "SPI.h"

  const static int DIAGx[] = {999, 1,  1, -1, -1, -1, -1,  1,  1};
  const static int DIAGy[] = {999, 1,  1,  1,  1, -1, -1, -1, -1};
  const static int SIDEx[] = {999, 1,  0,  0, -1, -1,  0,  0,  1};
  const static int SIDEy[] = {999, 0,  1,  1,  0,  0, -1, -1,  0};

GL_ST7735::GL_ST7735(uint8_t cs, uint8_t rs, uint8_t sid, 
  uint8_t sclk, uint8_t rst) {
  _cs = cs;
  _rs = rs;
  _sid = sid;
  _sclk = sclk;
  _rst = rst;
}

GL_ST7735::GL_ST7735(uint8_t cs, uint8_t rs,  uint8_t rst) {
  _cs = cs;
  _rs = rs;
  _sid = 0;
  _sclk = 0;
  _rst = rst;
}


inline void GL_ST7735::spiwrite(uint8_t c) {

  //Serial.println(c, HEX);

  if (!_sid) {
    SPI.transfer(c);
    return;
  }

  volatile uint8_t *sclkportreg = portOutputRegister(sclkport);
  volatile uint8_t *sidportreg = portOutputRegister(sidport);

  int8_t i;

  *sclkportreg |= sclkpin;

  for (i=7; i>=0; i--) {
    *sclkportreg &= ~sclkpin;
    //SCLK_PORT &= ~_BV(SCLK);
    
    if (c & _BV(i)) {
      *sidportreg |= sidpin;
      //digitalWrite(_sid, HIGH);
      //SID_PORT |= _BV(SID);
    } else {
      *sidportreg &= ~sidpin;
      //digitalWrite(_sid, LOW);
      //SID_PORT &= ~_BV(SID);
    }
    
    *sclkportreg |= sclkpin;
    //SCLK_PORT |= _BV(SCLK);
  }
}


void GL_ST7735::writecommand(uint8_t c) {
  *portOutputRegister(rsport) &= ~ rspin;
  //digitalWrite(_rs, LOW);

  *portOutputRegister(csport) &= ~ cspin;
  //digitalWrite(_cs, LOW);

  //Serial.print("C ");
  spiwrite(c);

  *portOutputRegister(csport) |= cspin;
  //digitalWrite(_cs, HIGH);
}


void GL_ST7735::writedata(uint8_t c) {
  *portOutputRegister(rsport) |= rspin;
  //digitalWrite(_rs, HIGH);

  *portOutputRegister(csport) &= ~ cspin;
    //digitalWrite(_cs, LOW);

  //Serial.print("D ");
  spiwrite(c);

  *portOutputRegister(csport) |= cspin;
  //digitalWrite(_cs, HIGH);
} 


void GL_ST7735::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  writecommand(ST7735_CASET);  // column addr set
  writedata(0x00);
  writedata(x0+0);   // XSTART 
  writedata(0x00);
  writedata(x1+0);   // XEND

  writecommand(ST7735_RASET);  // row addr set
  writedata(0x00);
  writedata(y0+0);    // YSTART
  writedata(0x00);
  writedata(y1+0);    // YEND

  writecommand(ST7735_RAMWR);  // write to RAM
}

void GL_ST7735::pushColor(uint16_t color) {
  *portOutputRegister(rsport) |= rspin;
  *portOutputRegister(csport) &= ~ cspin;

  spiwrite(color >> 8);    
  spiwrite(color);   

  *portOutputRegister(csport) |= cspin;
}

void GL_ST7735::drawPixel(uint8_t x, uint8_t y,uint16_t color) {
  if ((x >= width) || (y >= height)) return ;
  if ((x <= 0)     || (y <= 0)     ) return ;

  setAddrWindow(x,y,x+1,y+1);

  // setup for data
  *portOutputRegister(rsport) |= rspin;
  *portOutputRegister(csport) &= ~ cspin;

  spiwrite(color >> 8);    
  spiwrite(color);   

  *portOutputRegister(csport) |= cspin;

}


void GL_ST7735::fillScreen(uint16_t color) {
  setAddrWindow(0, 0, width-1, height-1);

  // setup for data
  *portOutputRegister(rsport) |= rspin;
  *portOutputRegister(csport) &= ~ cspin;

  for (uint8_t x=0; x < width; x++) {
    for (uint8_t y=0; y < height; y++) {
      spiwrite(color >> 8);    
      spiwrite(color);    
    }
  }

  *portOutputRegister(csport) |= cspin;
}

void GL_ST7735::initB(void) {
  // set pin directions
  pinMode(_rs, OUTPUT);

  if (_sclk) {
    pinMode(_sclk, OUTPUT);
    sclkport = digitalPinToPort(_sclk);
    sclkpin = digitalPinToBitMask(_sclk);

    pinMode(_sid, OUTPUT);
    sidport = digitalPinToPort(_sid);
    sidpin = digitalPinToBitMask(_sid);
  } else {
    // using the hardware SPI
    SPI.begin();
    SPI.setDataMode(SPI_MODE3);
  }
  // toggle RST low to reset; CS low so it'll listen to us
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, LOW);
  cspin = digitalPinToBitMask(_cs);
  csport = digitalPinToPort(_cs);

  rspin = digitalPinToBitMask(_rs);
  rsport = digitalPinToPort(_rs);

  if (_rst) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(500);
    digitalWrite(_rst, LOW);
    delay(500);
    digitalWrite(_rst, HIGH);
    delay(500);
  }

  writecommand(ST7735_SWRESET); // software reset
  delay(50);
  writecommand(ST7735_SLPOUT);  // out of sleep mode
  delay(500);
  
  writecommand(ST7735_COLMOD);  // set color mode
  writedata(0x05);        // 16-bit color
  delay(10);
  
  writecommand(ST7735_FRMCTR1);  // frame rate control
  writedata(0x00);  // fastest refresh
  writedata(0x06);  // 6 lines front porch
  writedata(0x03);  // 3 lines backporch
  delay(10);
  
  writecommand(ST7735_MADCTL);  // memory access control (directions)
  writedata(0x08);  // row address/col address, bottom to top refresh
  madctl = 0x08;

  writecommand(ST7735_DISSET5);  // display settings #5
  writedata(0x15);  // 1 clock cycle nonoverlap, 2 cycle gate rise, 3 cycle oscil. equalize
  writedata(0x02);  // fix on VTL
 
  writecommand(ST7735_INVCTR);  // display inversion control
  writedata(0x0);  // line inversion
 
  writecommand(ST7735_PWCTR1);  // power control
  writedata(0x02);      // GVDD = 4.7V 
  writedata(0x70);      // 1.0uA
  delay(10);
  writecommand(ST7735_PWCTR2);  // power control
  writedata(0x05);      // VGH = 14.7V, VGL = -7.35V 
  writecommand(ST7735_PWCTR3);  // power control
  writedata(0x01);      // Opamp current small 
  writedata(0x02);      // Boost frequency
  
  
  writecommand(ST7735_VMCTR1);  // power control
  writedata(0x3C);      // VCOMH = 4V
  writedata(0x38);      // VCOML = -1.1V
  delay(10);
  
  writecommand(ST7735_PWCTR6);  // power control
  writedata(0x11); 
  writedata(0x15);
  
  writecommand(ST7735_GMCTRP1);
  writedata(0x0f);	//writedata(0x09);  
  writedata(0x1a);  //writedata(0x16);
  writedata(0x0f);  //writedata(0x09);
  writedata(0x18);  //writedata(0x20);
  writedata(0x2f);  //writedata(0x21);
  writedata(0x28);  //writedata(0x1B);
  writedata(0x20);  //writedata(0x13);
  writedata(0x22);  //writedata(0x19);
  writedata(0x1f);  //writedata(0x17);
  writedata(0x1b);  //writedata(0x15);
  writedata(0x23);  //writedata(0x1E);
  writedata(0x37);  //writedata(0x2B);
  writedata(0x00);  //writedata(0x04);
  writedata(0x07);  //writedata(0x05);
  writedata(0x02);  //writedata(0x02);
  writedata(0x10);  //writedata(0x0E);
  writecommand(ST7735_GMCTRN1);
  writedata(0x0f);   //writedata(0x0B); 
  writedata(0x1b);   //writedata(0x14); 
  writedata(0x0f);   //writedata(0x08); 
  writedata(0x17);   //writedata(0x1E); 
  writedata(0x33);   //writedata(0x22); 
  writedata(0x2c);   //writedata(0x1D); 
  writedata(0x29);   //writedata(0x18); 
  writedata(0x2e);   //writedata(0x1E); 
  writedata(0x30);   //writedata(0x1B); 
  writedata(0x30);   //writedata(0x1A); 
  writedata(0x39);   //writedata(0x24); 
  writedata(0x3f);   //writedata(0x2B); 
  writedata(0x00);   //writedata(0x06); 
  writedata(0x07);   //writedata(0x06); 
  writedata(0x03);   //writedata(0x02); 
  writedata(0x10);   //writedata(0x0F); 
  delay(10);
  
  writecommand(ST7735_CASET);  // column addr set
  writedata(0x00);
  writedata(0x02);   // XSTART = 2
  writedata(0x00);
  writedata(0x81);   // XEND = 129

  writecommand(ST7735_RASET);  // row addr set
  writedata(0x00);
  writedata(0x02);    // XSTART = 1
  writedata(0x00);
  writedata(0x81);    // XEND = 160

  writecommand(ST7735_NORON);  // normal display on
  delay(10);
  
  writecommand(ST7735_RAMWR);
  delay(500);
  
  writecommand(ST7735_DISPON);
  delay(500);
}



void GL_ST7735::initR(void) {
  // set pin directions
  pinMode(_rs, OUTPUT);

  if (_sclk) {
    pinMode(_sclk, OUTPUT);
    sclkport = digitalPinToPort(_sclk);
    sclkpin = digitalPinToBitMask(_sclk);

    pinMode(_sid, OUTPUT);
    sidport = digitalPinToPort(_sid);
    sidpin = digitalPinToBitMask(_sid);
  } else {
    // using the hardware SPI
    SPI.begin();
    SPI.setDataMode(SPI_MODE3);
  }
  // toggle RST low to reset; CS low so it'll listen to us
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, LOW);
  cspin = digitalPinToBitMask(_cs);
  csport = digitalPinToPort(_cs);

  rspin = digitalPinToBitMask(_rs);
  rsport = digitalPinToPort(_rs);

  if (_rst) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(500);
    digitalWrite(_rst, LOW);
    delay(500);
    digitalWrite(_rst, HIGH);
    delay(500);
  }

 writecommand(ST7735_SWRESET); // software reset
  delay(150);

  writecommand(ST7735_SLPOUT);  // out of sleep mode
  delay(500);

  writecommand(ST7735_FRMCTR1);  // frame rate control - normal mode
  writedata(0x01);  // frame rate = fosc / (1 x 2 + 40) * (LINE + 2C + 2D)
  writedata(0x2C); 
  writedata(0x2D); 

  writecommand(ST7735_FRMCTR2);  // frame rate control - idle mode
  writedata(0x01);  // frame rate = fosc / (1 x 2 + 40) * (LINE + 2C + 2D)
  writedata(0x2C); 
  writedata(0x2D); 

  writecommand(ST7735_FRMCTR3);  // frame rate control - partial mode
  writedata(0x01); // dot inversion mode
  writedata(0x2C); 
  writedata(0x2D); 
  writedata(0x01); // line inversion mode
  writedata(0x2C); 
  writedata(0x2D); 
  
  writecommand(ST7735_INVCTR);  // display inversion control
  writedata(0x07);  // no inversion

  writecommand(ST7735_PWCTR1);  // power control
  writedata(0xA2);      
  writedata(0x02);      // -4.6V
  writedata(0x84);      // AUTO mode

  writecommand(ST7735_PWCTR2);  // power control
  writedata(0xC5);      // VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD

  writecommand(ST7735_PWCTR3);  // power control
  writedata(0x0A);      // Opamp current small 
  writedata(0x00);      // Boost frequency

  writecommand(ST7735_PWCTR4);  // power control
  writedata(0x8A);      // BCLK/2, Opamp current small & Medium low
  writedata(0x2A);     

  writecommand(ST7735_PWCTR5);  // power control
  writedata(0x8A);    
  writedata(0xEE);     

  writecommand(ST7735_VMCTR1);  // power control
  writedata(0x0E);  

  writecommand(ST7735_INVOFF);    // don't invert display

  writecommand(ST7735_MADCTL);  // memory access control (directions)
  writedata(0xC8);  // row address/col address, bottom to top refresh
  madctl = 0xC8;
  
  writecommand(ST7735_COLMOD);  // set color mode
  writedata(0x05);        // 16-bit color

  writecommand(ST7735_CASET);  // column addr set
  writedata(0x00);
  writedata(0x00);   // XSTART = 0
  writedata(0x00);
  writedata(0x7F);   // XEND = 127

  writecommand(ST7735_RASET);  // row addr set
  writedata(0x00);
  writedata(0x00);    // XSTART = 0
  writedata(0x00);
  writedata(0x9F);    // XEND = 159

  writecommand(ST7735_GMCTRP1);
  writedata(0x0f);
  writedata(0x1a);
  writedata(0x0f);
  writedata(0x18);
  writedata(0x2f);
  writedata(0x28);
  writedata(0x20);
  writedata(0x22);
  writedata(0x1f);
  writedata(0x1b);
  writedata(0x23);
  writedata(0x37);
  writedata(0x00);
  writedata(0x07);
  writedata(0x02);
  writedata(0x10);
  writecommand(ST7735_GMCTRN1);
  writedata(0x0f); 
  writedata(0x1b); 
  writedata(0x0f); 
  writedata(0x17); 
  writedata(0x33); 
  writedata(0x2c); 
  writedata(0x29); 
  writedata(0x2e); 
  writedata(0x30); 
  writedata(0x30); 
  writedata(0x39); 
  writedata(0x3f); 
  writedata(0x00); 
  writedata(0x07); 
  writedata(0x03); 
  writedata(0x10); 
  
  writecommand(ST7735_DISPON);
  delay(100);

  writecommand(ST7735_NORON);  // normal display on
  delay(10);
}


// draw a string from memory

void GL_ST7735::drawString(uint8_t x, uint8_t y, char const *c,
			uint16_t color, uint8_t size) {
  while (c[0] != 0) {
    drawChar(x, y, c[0], color, size);
    x += size*6;
    c++;
    if (x + 5 >= width) {
      y += 10;
      x = 0;
    }
  }
}
// draw a character
void GL_ST7735::drawChar(uint8_t x, uint8_t y, char c, 
		      uint16_t color, uint8_t size) {
  for (uint8_t i =0; i<5; i++ ) {
    uint8_t line = pgm_read_byte(font+(c*5)+i);
    for (uint8_t j = 0; j<8; j++) {
      if (line & 0x1) {
	if (size == 1) // default size
	  drawPixel(x+i, y+j, color);
	else {  // big size
	  fillRect(x+i*size, y+j*size, size, size, color);
	} 
      }
      line >>= 1;
    }
  }
}

// fill a circle
void GL_ST7735::fillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawVerticalLine(x0, y0-r, 2*r+1, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    drawVerticalLine(x0+x, y0-y, 2*y+1, color);
    drawVerticalLine(x0-x, y0-y, 2*y+1, color);
    drawVerticalLine(x0+y, y0-x, 2*x+1, color);
    drawVerticalLine(x0-y, y0-x, 2*x+1, color);
  }
}

// draw a circle outline
void GL_ST7735::drawCircle(uint8_t x0, uint8_t y0, uint8_t r, 
			uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawPixel(x0, y0+r, color);
  drawPixel(x0, y0-r, color);
  drawPixel(x0+r, y0, color);
  drawPixel(x0-r, y0, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
    
  }
}

uint8_t GL_ST7735::getRotation() {
  return madctl;
}

void GL_ST7735::setRotation(uint8_t m) {
  madctl = m;
  writecommand(ST7735_MADCTL);  // memory access control (directions)
  writedata(madctl);  // row address/col address, bottom to top refresh
}

// draw a rectangle
void GL_ST7735::drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		      uint16_t color) {
  // smarter version
  drawHorizontalLine(x, y, w, color);
  drawHorizontalLine(x, y+h-1, w, color);
  drawVerticalLine(x, y, h, color);
  drawVerticalLine(x+w-1, y, h, color);
}

void GL_ST7735::fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		      uint16_t color) {
  // smarter version

  setAddrWindow(x, y, x+w-1, y+h-1);

  // setup for data
  digitalWrite(_rs, HIGH);
  digitalWrite(_cs, LOW);

  for (x=0; x < w; x++) {
    for (y=0; y < h; y++) {
      spiwrite(color >> 8);    
      spiwrite(color);    
    }
  }
  digitalWrite(_cs, HIGH);
}

void GL_ST7735::drawVerticalLine(uint8_t x, uint8_t y, uint8_t length, uint16_t color)
{
  if (x >= width) return;
  if (y+length >= height) length = height-y-1;

  drawFastLine(x,y,length,color,1);
}

void GL_ST7735::drawHorizontalLine(uint8_t x, uint8_t y, uint8_t length, uint16_t color)
{
  if (y >= height) return;
  if (x+length >= width) length = width-x-1;

  drawFastLine(x,y,length,color,0);
}

void GL_ST7735::drawFastLine(uint8_t x, uint8_t y, uint8_t length, 
			  uint16_t color, uint8_t rotflag)
{
  if (rotflag) {
    setAddrWindow(x, y, x, y+length);
  } else {
    setAddrWindow(x, y, x+length, y+1);
  }
  // setup for data
  digitalWrite(_rs, HIGH);
  digitalWrite(_cs, LOW);

  while (length--) {
    spiwrite(color >> 8);    
    spiwrite(color);    
  }
  digitalWrite(_cs, HIGH);
}


// bresenham's algorithm - thx wikpedia
void GL_ST7735::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, 
		      uint16_t color) {
  uint16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  uint16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;}

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void GL_ST7735::assign(long A_, long B_, long C_, long D_, long E_, long F_) {
  A = A_;
  B = B_;
  C = C_;
  D = D_;
  E = E_;
  F = F_;
}

void GL_ST7735::assignf(double scale,
                           double A_,
                           double B_,
                           double C_,
                           double D_,
                           double E_,
                           double F_) {
  A = rnd(A_ * scale);
  B = rnd(B_ * scale);
  C = rnd(C_ * scale);
  D = rnd(D_ * scale);
  E = rnd(E_ * scale);
  F = rnd(F_ * scale);
}

long GL_ST7735::getA() { return A ;}
long GL_ST7735::getB() { return B ;}
long GL_ST7735::getC() { return C ;}
long GL_ST7735::getD() { return D ;}
long GL_ST7735::getE() { return E ;}
long GL_ST7735::getF() { return F ;}


inline long GL_ST7735::rnd(double x) {
	return (x>=0.0)?(long)(x + 0.5):(long)(x - 0.5);
}

/*
 * drawConic
 *
 *  Created on: Oct 19, 2015
 *      Author: vaj4088 - Ian Shef
 *
 *     Modified from code by:
 */
// Author: Andrew W. Fitzgibbon (andrewfg@ed.ac.uk),
//         Machine Vision Unit,
//         Dept. of Artificial Intelligence,
//         Edinburgh University,
//
// Date: 31-Mar-94
// Version 2: 6-Oct-95
//      Bugfixes from Arne Steinarson <arst@ludd.luth.se>
//

//  Bresenham Ellipse Algorithm

//
// drawConic  2D Bresenham-like conic drawer.
//       CONIC(Sx,Sy, Ex,Ey, A,B,C,D,E,F) draws the conic specified
//       by A x^2 + B x y + C y^2 + D x + E y + F = 0, between the
//       start point (Sx, Sy) and endpoint (Ex,Ey).
//
//  NOTE 1:  There is no Sx, Sy, Ex, or Ey in the constructor or
//           the method assign(...), but they show up in the parameters
//           for actually drawing the conic.
//
//  NOTE 2:  An ellipse centered at (0,0) is given by
//           A x^2 + C y^2 + F = 0
//           where F = -(A C)
//
//           The radius in the x direction is given by the square root of C.
//           The radius in the y direction is given by the square root of A.
//
//  NOTE 3:  The general case for an ellipse centered at (X_c, y_c),
//           semi-major axis a, semi-minor axis b,
//           and rotation angle Theta :
//
//
// A = a^2 (sin(Theta))^2 + b^2 (cos(Theta))^2
// B = 2 (b^2-a^2) sin(Theta) cos(Theta)
// C = a^2 (cos(Theta))^2 + b^2 (sin(Theta))^2
// D = -2 A x_c - B y_c
// E = -B x_c - 2 C y_c
// F = A (x_c)^2 + B x_c y_c + C (y_c)^2 - a^2 b^2
//
// from https://en.wikipedia.org/wiki/Ellipse
// under "General ellipse".
//

void GL_ST7735::drawConicHelper(int xs, int ys, int xe, int ye, int color)
{
#if (DEBUG)
  {
  const char *fmt =
		  "GL_ST7735::drawConicHelper called with %7d %7d %7d %7d %7d" ;
  char buf[snprintf(NULL, 0, fmt, xs, ys, xe, ye, color) + 1] ;
//
//  note +1 for terminating null byte
//
  snprintf(buf, sizeof buf, fmt, xs, ys, xe, ye, color) ;
  Serial.println(buf) ;
  }
#endif
#if (DEBUG)
  {
  const char *fmt =
		  "GL_ST7735::drawConicHelper -1- %12ld %12ld %12ld %12ld %12ld %12ld" ;
  char buf[snprintf(NULL, 0, fmt, A,B,C,D,E,F) + 1] ;
//
//  note +1 for terminating null byte
//
  snprintf(buf, sizeof buf, fmt, A,B,C,D,E,F) ;
  Serial.println(buf) ;
  }
#endif

  A *= 4;
  B *= 4;
  C *= 4;
  D *= 4;
  E *= 4;
  F *= 4;

#if (DEBUG)
  {
  const char *fmt =
		  "GL_ST7735::drawConicHelper -2- %12ld %12ld %12ld %12ld %12ld %12ld" ;
  char buf[snprintf(NULL, 0, fmt, A,B,C,D,E,F) + 1] ;
//
//  note +1 for terminating null byte
//
  snprintf(buf, sizeof buf, fmt, A,B,C,D,E,F) ;
  Serial.println(buf) ;
  }
#endif

  // Translate start point to origin...
  F = A*xs*xs + B*xs*ys + C*ys*ys + D*xs + E*ys + F;
  D = D + 2 * A * xs + B * ys;
  E = E + B * xs + 2 * C * ys;

  // Work out starting octant
  int octant = getOctant(D,E);

  int dxS = SIDEx[octant];
  int dyS = SIDEy[octant];
  int dxD = DIAGx[octant];
  int dyD = DIAGy[octant];

  long d,u,v;

#if (DEBUG)
  {
  const char *fmt = "Before switch, octant = %d" ;
  char buf[snprintf(NULL, 0, fmt, octant) + 1] ;
//
//  note +1 for terminating null byte
//
  snprintf(buf, sizeof buf, fmt, octant) ;
  Serial.println(buf) ;
  }
#endif

	if (octant == 1) {
		d = A + B / 2 + C / 4 + D + E / 2 + F;
		u = A + B / 2 + D;
		v = u + E;
	} else if (octant == 2) {
		d = A / 4 + B / 2 + C + D / 2 + E + F;
		u = B / 2 + C + E;
		v = u + D;
	} else if (octant == 3) {
		d = A / 4 - B / 2 + C - D / 2 + E + F;
		u = -B / 2 + C + E;
		v = u - D;
	} else if (octant == 4) {
		d = A - B / 2 + C / 4 - D + E / 2 + F;
		u = A - B / 2 - D;
		v = u + E;
	} else if (octant == 5) {
		d = A + B / 2 + C / 4 - D - E / 2 + F;
		u = A + B / 2 - D;
		v = u - E;
	} else if (octant == 6) {
		d = A / 4 + B / 2 + C - D / 2 - E + F;
		u = B / 2 + C - E;
		v = u - D;
	} else if (octant == 7) {
		d = A / 4 - B / 2 + C + D / 2 - E + F;
		u = -B / 2 + C - E;
		v = u + D;
	} else if (octant == 8) {
		d = A - B / 2 + C / 4 + D - E / 2 + F;
		u = A - B / 2 + D;
		v = u - E;
	} else {
		d=0 ; u=0 ; v=0 ;
		const char *fmt = "FUNNY OCTANT";
		char buf[snprintf(NULL, 0, fmt) + 1];
		//
		//  note +1 for terminating null byte
		//
		snprintf(buf, sizeof buf, fmt);
		Serial.println(buf);
		while (true) {} ;
	}

//	switch (octant) {
//	case 1:
//		d = A + B / 2 + C / 4 + D + E / 2 + F;
//		u = A + B / 2 + D;
//		v = u + E;
//		break;
//	case 2:
//		d = A / 4 + B / 2 + C + D / 2 + E + F;
//		u = B / 2 + C + E;
//		v = u + D;
//		break;
//	case 3:
//		d = A / 4 - B / 2 + C - D / 2 + E + F;
//		u = -B / 2 + C + E;
//		v = u - D;
//		break;
//	case 4:
//		d = A - B / 2 + C / 4 - D + E / 2 + F;
//		u = A - B / 2 - D;
//		v = u + E;
//		break;
//	case 5:
//		d = A + B / 2 + C / 4 - D - E / 2 + F;
//		u = A + B / 2 - D;
//		v = u - E;
//		break;
//	case 6:
//		d = A / 4 + B / 2 + C - D / 2 - E + F;
//		u = B / 2 + C - E;
//		v = u - D;
//		break;
//	case 7:
//		d = A / 4 - B / 2 + C + D / 2 - E + F;
//		u = -B / 2 + C - E;
//		v = u + D;
//		break;
//	case 8:
//		d = A - B / 2 + C / 4 + D - E / 2 + F;
//		u = A - B / 2 + D;
//		v = u - E;
//		break;
//	default:
//		d=0 ; u=0 ; v=0 ;
//		const char *fmt = "FUNNY OCTANT";
//		char buf[snprintf(NULL, 0, fmt) + 1];
//		//
//		//  note +1 for terminating null byte
//		//
//		snprintf(buf, sizeof buf, fmt);
//		Serial.println(buf);
//		while (true) {} ;
//		break ;
//	}

#if (DEBUG)
  {
  const char *fmt = "After  switch, octant = %d" ;
  char buf[snprintf(NULL, 0, fmt, octant) + 1] ;
//
//  note +1 for terminating null byte
//
  snprintf(buf, sizeof buf, fmt, octant) ;
  Serial.println(buf) ;
  }
#endif

  long k1sign = dyS*dyD;
  long k1 = 2 * (A + k1sign * (C - A));
  long Bsign = dxD*dyD;
  long k2 = k1 + Bsign * B;
  long k3 = 2 * (A + C + Bsign * B);

  // Work out gradient at endpoint
  long gxe = xe - xs;
  long gye = ye - ys;
  long gx = 2*A*gxe +   B*gye + D;
  long gy =   B*gxe + 2*C*gye + E;

  int octantCount = getOctant(gx,gy) - octant;
  if (octantCount < 0)
    octantCount = octantCount + 8;
  else if (octantCount==0)
    if((xs>xe && dxD>0) || (ys>ye && dyD>0) ||
       (xs<xe && dxD<0) || (ys<ye && dyD<0))
      octantCount +=8;

#if (DEBUG)
  {
  const char *fmt = "octantCount = %d\n" ;
  char buf[snprintf(NULL, 0, fmt, octantCount) + 1] ;
//
//  note +1 for terminating null byte
//
  snprintf(buf, sizeof buf, fmt, octantCount) ;
  Serial.print(buf) ;
  }
#endif

  long x = xs;
  long y = ys;

  while (octantCount > 0) {
	#if (DEBUG)
	  {
	  const char *fmt = "-- %d -------------------------\n" ;
	  char buf[snprintf(NULL, 0, fmt, octant) + 1] ;
	//
	//  note +1 for terminating null byte
	//
	  snprintf(buf, sizeof buf, fmt, octant) ;
	  Serial.print(buf) ;
	  }
	#endif

    if (odd(octant)) {
      while (2*v <= k2) {
        // Plot this point
    	  drawPixel(x, y, color) ;

        // Are we inside or outside?

		#if (DEBUG)
		  {
		  const char *fmt = "x = %ld y = %ld d = %ld\n" ;
		  char buf[snprintf(NULL, 0, fmt, x,y,d) + 1] ;
		//
		//  note +1 for terminating null byte
		//
		  snprintf(buf, sizeof buf, fmt, x,y,d) ;
		  Serial.print(buf) ;
		  }
		#endif

        if (d < 0) {                    // Inside
          x = x + dxS;
          y = y + dyS;
          u = u + k1;
          v = v + k2;
          d = d + u;
        }
        else {                          // outside
          x = x + dxD;
          y = y + dyD;
          u = u + k2;
          v = v + k3;
          d = d + v;
        }
      }

      d = d - u + v/2 - k2/2 + 3*k3/8;
      // error (^) in Foley and van Dam p 959, "2nd ed, revised 5th printing"
      u = -u + v - k2/2 + k3/2;
      v = v - k2 + k3/2;
      k1 = k1 - 2*k2 + k3;
      k2 = k3 - k2;
      int tmp = dxS; dxS = -dyS; dyS = tmp;
    }
    else {                              // Octant is even
      while (2*u < k2) {
        // Plot this point
    	  drawPixel(x, y, color) ;

		#if (DEBUG)
		  {
		  const char *fmt = "x = %ld y = %ld d = %ld\n" ;
		  char buf[snprintf(NULL, 0, fmt, x,y,d) + 1] ;
		//
		//  note +1 for terminating null byte
		//
		  snprintf(buf, sizeof buf, fmt, x,y,d) ;
		  Serial.print(buf) ;
		  }
		#endif

        // Are we inside or outside?
        if (d > 0) {                    // Outside
          x = x + dxS;
          y = y + dyS;
          u = u + k1;
          v = v + k2;
          d = d + u;
        }
        else {                          // Inside
          x = x + dxD;
          y = y + dyD;
          u = u + k2;
          v = v + k3;
          d = d + v;
        }
      }
      long tmpdk = k1 - k2;
      d = d + u - v + tmpdk;
      v = 2*u - v + tmpdk;
      u = u + tmpdk;
      k3 = k3 + 4*tmpdk;
      k2 = k1 + tmpdk;

      int tmp = dxD; dxD = -dyD; dyD = tmp;
    }

    octant = (octant&7)+1;
    octantCount--;
  }

  // Draw final octant until we reach the endpoint

	#if (DEBUG)
	  {
	  const char *fmt = "-- %d (final) -----------------\n" ;
	  char buf[snprintf(NULL, 0, fmt, octant) + 1] ;
	//
	//  note +1 for terminating null byte
	//
	  snprintf(buf, sizeof buf, fmt, octant) ;
	  Serial.print(buf) ;
	  }
	#endif

  if (odd(octant)) {
    while (2*v <= k2) {
      // Plot this point
    	drawPixel(x, y, color) ;
      if (x == xe && y == ye)
        break;

		#if (DEBUG)
		  {
		  const char *fmt = "x = %ld y = %ld d = %ld\n" ;
		  char buf[snprintf(NULL, 0, fmt, x,y,d) + 1] ;
		//
		//  note +1 for terminating null byte
		//
		  snprintf(buf, sizeof buf, fmt, x,y,d) ;
		  Serial.print(buf) ;
		  }
		#endif

      // Are we inside or outside?
      if (d < 0) {                      // Inside
        x = x + dxS;
        y = y + dyS;
        u = u + k1;
        v = v + k2;
        d = d + u;
      }
      else {                            // outside
        x = x + dxD;
        y = y + dyD;
        u = u + k2;
        v = v + k3;
        d = d + v;
      }
    }
  }
  else {                                // Octant is even
    while ((2*u < k2)) {
      // Plot this point
    	drawPixel(x, y, color) ;
      if (x == xe && y == ye)
        break;

		#if (DEBUG)
		  {
		  const char *fmt = "x = %ld y = %ld d = %ld\n" ;
		  char buf[snprintf(NULL, 0, fmt, x,y,d) + 1] ;
		//
		//  note +1 for terminating null byte
		//
		  snprintf(buf, sizeof buf, fmt, x,y,d) ;
		  Serial.print(buf) ;
		  }
		#endif

      // Are we inside or outside?
      if (d > 0) {                      // Outside
        x = x + dxS;
        y = y + dyS;
        u = u + k1;
        v = v + k2;
        d = d + u;
      }
      else {                            // Inside
        x = x + dxD;
        y = y + dyD;
        u = u + k2;
        v = v + k3;
        d = d + v;
      }
    }
  }
}



void GL_ST7735::drawConic(int xs, int ys, int xe, int ye, int color)
{
	//
	//  Save parameters before modification.
	//
	long A9 = A ;
	long B9 = B ;
	long C9 = C ;
	long D9 = D ;
	long E9 = E ;
	long F9 = F ;

	drawConicHelper(xs, ys, xe, ye, color) ;

  //
  // Restore parameters.
  //
	A = A9 ; B = B9 ; C = C9 ; D = D9 ; E = E9 ; F = F9 ;
}

void GL_ST7735::drawEllipse(int xc,
		int yc,
		int semiMajor,
		int semiMinor,
		int theta,
		int color) {
	/*
	 * A = a^2 (sin(Theta))^2 + b^2 (cos(Theta))^2
     * B = 2 (b^2-a^2) sin(Theta) cos(Theta)
     * C = a^2 (cos(Theta))^2 + b^2 (sin(Theta))^2
     * D = -2 A x_c - B y_c
     * E = -B x_c - 2 C y_c
     * F = A (x_c)^2 + B x_c y_c + C (y_c)^2 - a^2 b^2
     *
     * x_can = (x-x_c) cos(Theta) + (y-y_c) sin(Theta)
     *
     * x_can = x canonical
     *
	 */
	double twoPi = 8.0 * atan(1.0) ;
	double sinTheta = sin((twoPi * theta) / 360.0) ;
	double cosTheta = cos((twoPi * theta) / 360.0) ;
	double sinThetaSquared = sinTheta * sinTheta ;
	double cosThetaSquared = cosTheta * cosTheta ;
	double aSquared = (double)semiMajor * (double)semiMajor ;
	double bSquared = (double)semiMinor * (double)semiMinor ;
	double A = aSquared * sinThetaSquared + bSquared * cosThetaSquared ;
	double B = (double)2.0 * sinTheta * cosTheta * (bSquared - aSquared) ;
	double C = aSquared * cosThetaSquared + bSquared * sinThetaSquared ;
	double D = (double)(-2.0) * A * xc - B * yc ;
	double E = (double)(-2.0) * C * yc - B * xc ;
	double F = A * xc * xc + B * xc * yc + C * yc * yc - aSquared * bSquared ;
	assignf(1.0, A, B, C, D, E, F) ;
	double xStart = (double)xc + (double)semiMajor * cosTheta ;
	double yStart = (double)yc + (double)semiMajor * sinTheta ;
	double xEnd   = (double)xc - (double)semiMajor * cosTheta ;
	double yEnd   = (double)yc - (double)semiMajor * sinTheta ;

	assign(A, B, C, D, E, F) ;

	drawConicHelper(
			(int)rnd(xStart),
			(int)rnd(yStart),
			(int)rnd(xEnd),
			(int)rnd(yEnd),
			color) ;

	//
	// Reset parameters.
	//
	assign(A, B, C, D, E, F) ;

	//
	// Notice that Start and End have been reversed in the following call:
	//
	drawConicHelper(
			(int)rnd(xEnd),
			(int)rnd(yEnd),
			(int)rnd(xStart),
			(int)rnd(yStart),
			color) ;

	//
	// Restore parameters.
	//
	assign(A, B, C, D, E, F) ;
}

inline int GL_ST7735::odd(int n)
{
  return n&1;
}


int GL_ST7735::getOctant(long gx, long gy)
{
  // Use gradient to identify octant.
  int upper = abs(gx)>abs(gy);
  if (gx>=0)                            // Right-pointing
    if (gy>=0)                          //    Up
      return 4 - upper;
    else                                //    Down
      return 1 + upper;
  else                                  // Left
    if (gy>0)                           //    Up
      return 5 + upper;
    else                                //    Down
      return 8 - upper;
}

void GL_ST7735::setInverted(boolean b) {
	if (b) {
		writecommand(ST7735_INVON);
	} else {
		writecommand(ST7735_INVOFF);
	}
}


//////////
/*
uint8_t GL_ST7735::spiread(void) {
  uint8_t r = 0;
  if (_sid > 0) {
    r = shiftIn(_sid, _sclk, MSBFIRST);
  } else {
    //SID_DDR &= ~_BV(SID);
    //int8_t i;
    //for (i=7; i>=0; i--) {
    //  SCLK_PORT &= ~_BV(SCLK);
    //  r <<= 1;
    //  r |= (SID_PIN >> SID) & 0x1;
    //  SCLK_PORT |= _BV(SCLK);
    //}
    //SID_DDR |= _BV(SID);
    
  }
  return r;
}



void GL_ST7735::dummyclock(void) {

  if (_sid > 0) {
    digitalWrite(_sclk, LOW);
    digitalWrite(_sclk, HIGH);
  } else {
    // SCLK_PORT &= ~_BV(SCLK);
    //SCLK_PORT |= _BV(SCLK);
  }
}
uint8_t GL_ST7735::readdata(void) {
  *portOutputRegister(rsport) |= rspin;

  *portOutputRegister(csport) &= ~ cspin;

  uint8_t r = spiread();

  *portOutputRegister(csport) |= cspin;

  return r;

} 

uint8_t GL_ST7735::readcommand8(uint8_t c) {
  digitalWrite(_rs, LOW);

  *portOutputRegister(csport) &= ~ cspin;

  spiwrite(c);

  digitalWrite(_rs, HIGH);
  pinMode(_sid, INPUT); // input!
  digitalWrite(_sid, LOW); // low
  spiread();
  uint8_t r = spiread();


  *portOutputRegister(csport) |= cspin;


  pinMode(_sid, OUTPUT); // back to output
  return r;
}


uint16_t GL_ST7735::readcommand16(uint8_t c) {
  digitalWrite(_rs, LOW);
  if (_cs)
    digitalWrite(_cs, LOW);

  spiwrite(c);
  pinMode(_sid, INPUT); // input!
  uint16_t r = spiread();
  r <<= 8;
  r |= spiread();
  if (_cs)
    digitalWrite(_cs, HIGH);

  pinMode(_sid, OUTPUT); // back to output
  return r;
}

uint32_t GL_ST7735::readcommand32(uint8_t c) {
  digitalWrite(_rs, LOW);
  if (_cs)
    digitalWrite(_cs, LOW);
  spiwrite(c);
  pinMode(_sid, INPUT); // input!

  dummyclock();
  dummyclock();

  uint32_t r = spiread();
  r <<= 8;
  r |= spiread();
  r <<= 8;
  r |= spiread();
  r <<= 8;
  r |= spiread();
  if (_cs)
    digitalWrite(_cs, HIGH);

  pinMode(_sid, OUTPUT); // back to output
  return r;
}

*/
