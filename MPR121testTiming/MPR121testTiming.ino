/*********************************************************
This is a library for the MPR121 12-channel Capacitive touch sensor

Designed specifically to work with the MPR121 Breakout in the Adafruit shop 
  ----> https://www.adafruit.com/products/

These sensors use I2C communicate, at least 2 pins are required 
to interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.  
BSD license, all text above must be included in any redistribution
**********************************************************/

#include <Wire.h>
#include "Adafruit_MPR121.h"

#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
char buttonPressed = 0;
uint16_t count = 0;// use this to tell which button pressed first
uint16_t LButton = 1;// Left button is the positive end
uint16_t NButton = 0;// N button will be negative
char packet[8];
//packet[1] = 255;
unsigned long tick,tock,lastTouched = 0;
unsigned long ave,sum =0;

int countL,countR = 0;
int start,stopBit = 0;


int aveCount = 0;

void setup() {
  Serial.begin(115200);
  packet[0] = 255;

  while (!Serial) { // needed to keep leonardo/micro from starting too fast!
    delay(10);
  }
    
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    //Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  //Serial.println("MPR121 found!");
  //Serial.write(char(0xFF));
}

void loop() {
  // Get the currently touched pads
  currtouched = cap.touched();
char dat[8];
  if(!stopBit){
  for (uint8_t i=0; i<3; i++) { // can make this only loop through 0-2
    if(start==1 && (((3*ave) + lastTouched) < micros())) {
      //Serial.println("TIMEOUT");
    }

    
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      
      //Serial.print(i); Serial.println(" touched");
      Serial.write(0xFF);
      Serial.write(0xFF);
      Serial.write(i&0xFF);
      Serial.write(0x00);
      sprintf(dat, "%08lx", (unsigned long)micros() & 0xFFFFFFFF);
      //Serial.println("");
      Serial.write(dat);
      
      if(0 == count){
        tick = micros();
        lastTouched = tick;
        count++;
        buttonPressed = i;
      }
      else if(1 == count && (i != buttonPressed)){
        start =1;
        tock = micros();
        lastTouched = tock;
        //Serial.print("Time Difference: ");
        Serial.write(0xFF);
        Serial.write(0xFF);
        Serial.write(0x03);
        if(i == NButton){
          Serial.write(0x00);
        } else if (i == LButton){
          Serial.write(0x01);
        }
        unsigned long res = tock-tick;
        sprintf(dat, "%08lx", (unsigned long)(res) & 0xFFFFFFFF);
        Serial.write(dat);
        sum += res;
        aveCount+=1;
        ave = sum/aveCount;
      
        
        count = 0;
      }
      else if(1 == count && (i == buttonPressed)){
        count = 1;
        buttonPressed = i;//throw out this button press
      }
      
    }
  }
  }

  // reset our state
  lasttouched = currtouched;

  // comment out this line for detailed data from the sensor!
  return;
  
  // debugging info, what
  Serial.print("\t\t\t\t\t\t\t\t\t\t\t\t\t 0x"); Serial.println(cap.touched(), HEX);
  Serial.print("Filt: ");
  for (uint8_t i=0; i<12; i++) {
    Serial.print(cap.filteredData(i)); Serial.print("\t");
  }
  Serial.println();
  Serial.print("Base: ");
  for (uint8_t i=0; i<12; i++) {
    Serial.print(cap.baselineData(i)); Serial.print("\t");
  }
  Serial.println();
  
  // put a delay so it isn't overwhelming
  delay(100);
}
