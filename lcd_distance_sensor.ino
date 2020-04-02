/*
 * This Arduino sketch display distance measured 
 * using HC-SR04  as bargraph on LCD1602-I2C
 * 
 * Original library was taken from https://playground.arduino.cc/Code/LcdBarGraph/
 * Modified by Ahmad Shamshiri on May 26, 2019 at 04:53 in Ajax, Ontario, Canada
 * Visit http://robojax.com/learn/arduino for other Arduino codes
 * Watch YouTube video instruction for this code: https://youtu.be/V-o32As_Xv4
 * 
 * Robojax Arduino Course on Udemy where  you get Schematic Diagram of this sketch 
 * and many other robotics related lectures and codes. Purchase the course here: http://bit.ly/rj-udemy
 * 
 
 * This code is "AS IS" without warranty or liability. Free to be used as long as you keep this note intact.* 
 * This code has been download from Robojax.com
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // LiquidCrystal_I2C Library
#include <LcdBarGraph.h> // LcdBarGraph Library


byte lcdNumCols = 16; // -- number of columns in the LCD
byte lcdLine = 2; // -- number of line in the LCD

LiquidCrystal_I2C lcd(0x27,lcdNumCols,lcdLine); //0x3f is address for I2C
                  // to get I2C address, run I2C Scanner. 
                  //Link is provided (in same page as this code) at http://robojax.com/learn/arduino
LcdBarGraphRobojax rjx(&lcd, 16, 0, 0);  // -- creating 16 character long bargraph starting at char 0 of line 0 (see video)

#include <NewPing.h>// NewPing Library for Distance Sensor

#define TRIGGER_PIN  11  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimetres). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
int VCC2 = 2;// additional VCC(5) for device
void setup(){
  pinMode(VCC2, OUTPUT);// extra VCC for ultrasonic
  digitalWrite(VCC2, HIGH);//extra VCC is now HIGH (5V)
    
  // -- initializing the LCD
  lcd.begin();
  lcd.clear();
  lcd.print("Spritle Tech"); 
  lcd.setCursor (0,1); //  
  lcd.print("LCD-Bargraph"); 
   
  // -- do some delay: some time I've got broken visualization
  delay(2000);
  lcd.clear();
}

void loop()
{
  int distanceCM = sonar.ping_cm();// get distance in cm
  int distanceIN = sonar.ping_in();// get distance in in
 
  rjx.clearLine(1);// clear line 1 to display fresh voltage value

  // -- draw bar graph from the analog value 
  rjx.drawValue( distanceCM, MAX_DISTANCE);
  // -- do some delay: frequent draw may cause broken visualization
 
  lcd.setCursor (0,1); //
  lcd.print("Distance:"); 
  lcd.setCursor (10,1); //
  lcd.print(distanceCM); // print
  lcd.setCursor (14,1); //  
  lcd.print("cm");   
 
  delay(100);
}