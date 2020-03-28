/*
 * 
 * MAX30112 Library by Joshua Brewster.
 * Based on the Sparkfun MAX30105 library for compatibility with the free HR and SPO2 libraries.
 * 
 * License: MIT (2020). Do whatever with it, really.
 */

#include <Wire.h>
#include "MAX30112.h"

MAX30112 pulseOx;

#define debug Serial //Uncomment this line if you're using an Uno or ESP
//#define debug SerialUSB //Uncomment this line if you're using a SAMD21

#define SDA 5
#define SCL 18
#define INT 23

void setup()
{

  debug.begin(115200);
  debug.println("MAX30112 Basic Readings Example");
  Wire.begin(SDA,SCL);
  // Initialize sensor
  if (pulseOx.begin() == false)
  {
    debug.println("MAX30112 was not found. Please check wiring/power. ");
    while (1);
  }
  pulseOx.setup(); //Configure sensor with default parameters. See code for config options
 
}

void loop()
{
  debug.print(" R[");
  debug.print(pulseOx.getFIFORed());
  debug.print("] IR[");
  debug.print(pulseOx.getFIFOIR());
  debug.print("]");

  debug.println();
  delay(1000);
}
