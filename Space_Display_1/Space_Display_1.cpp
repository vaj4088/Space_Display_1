// Do not remove the include below
#include "Space_Display_1.h"

const byte CS    = 10 ;
const byte DC    =  9 ;
const byte RESET =  8 ;

GL_ST7735 lcd = GL_ST7735(CS, DC, RESET);


//The setup function is called once at startup of the sketch
void setup()
{
// Add your initialization code here
	Serial.begin(115200) ;
	lcd.initR() ;
	clearScreen(lcd) ;  //  Clear screen.
	byte radius = lcd.width/2 -1 ;
	lcd.fillCircle(lcd.width/2, lcd.height/2, radius  , DARKGREEN) ;
	lcd.fillCircle(lcd.width/2, lcd.height/2, radius-6, LIGHTBLUE) ;
	lcd.drawString(22, 65, "EFH", BLACK, 5) ;
}

// The loop function is called in an endless loop
void loop() {
	delay(2000) ;
	lcd.setInverted(true) ;
	delay(2000) ;
	lcd.setInverted(false) ;
}

// NOTE:
//        B = 5 bits (most significant)
//        G = 6 bits (middle)
//        R = 5 bits (least significant)

void clearScreen(GL_ST7735 obj) {
	//
	//  Clear screen to light blue.
	//
	obj.fillRect(0, 0, lcd.width, lcd.height, LIGHTBLUE) ;
}

