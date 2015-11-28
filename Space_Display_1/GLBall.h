/*
 * GLBall.h
 *
 *  Created on: Oct 12, 2015
 *      Author: Ian Shef
 */


#ifndef GLBALL_H_
#define GLBALL_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <GL_ST7735.h>

// NOTE:
//        B = 5 bits (most significant)
//        G = 6 bits (middle)
//        R = 5 bits (least significant)

const unsigned int BLACK  = 0x0000 ;
const unsigned int RED    = 0x001F ;
const unsigned int GREEN  = 0x07E0 ;
const unsigned int BLUE   = 0xF800 ;
const unsigned int WHITE  = ~BLACK ;
const unsigned int MAGENTA= ~GREEN ;
const unsigned int YELLOW = ~BLUE  ;
const unsigned int CYAN   = ~RED   ;
const unsigned int DARKGREEN = 0x0380 ;
const unsigned int LIGHTBLUE = 0xFD9B ;


class GLBall {
public:
	GLBall(GL_ST7735 *st7735) ;
	unsigned int  getBallColor()  const;
	unsigned long getFrameTime()  const;
	unsigned int  getRadius()     const;
	unsigned int  getTrailColor() const;
	unsigned int  getXCurrent()   const;
	unsigned int  getXPrevious()  const;
	         int  getXVel()       const;
	unsigned int  getYCurrent()   const;
	unsigned int  getYPrevious()  const;
	         int  getYVel()       const;
	void begin() ;
    boolean update() ;  // returns true if updated, false otherwise.
    GLBall& setBallColor(unsigned int ballColor);
	GLBall& setFrameTime(unsigned long frameTime);
	GLBall& setRadius(int radius);
	GLBall& setTrailColor(unsigned int trailColor);
	GLBall& setXCurrent(int current);
	GLBall& setXPrevious(int previous);
	GLBall& setXVel(int vel);
	GLBall& setYCurrent(int current);
	GLBall& setYPrevious(int previous);
	GLBall& setYVel(int vel);
private:
	//
	// Ball parameters
	//
	unsigned long frameTime ;  //  The time from one animation frame
	                           //  to the next in milliseconds.
	unsigned int radius ;
	unsigned int xCurrent ;
	unsigned int yCurrent ;
	unsigned int ballColor ;
	unsigned int trailColor ;
	unsigned int xPrevious ;
	unsigned int yPrevious ;
	//
	// Velocities are in pixels per frame.
	//
	int xVel ;
	int yVel ;
	unsigned long currentTime ;
	unsigned long previousTime ;
	GL_ST7735 *lcd ;

};

#endif /* GLBALL_H_ */
