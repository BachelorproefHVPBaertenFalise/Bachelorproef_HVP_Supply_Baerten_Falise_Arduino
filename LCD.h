#include <SPI.h>
#include "Arduino.h"

#ifndef LCD_H
#define LCD_H

/*
   Authors: Sven Baerten (sven.baerten@student.uhasselt.be) &
            Wouter Falise (wouter.falise@student.uhasselt.be)

   Library for the LCD display made by "NEWHAVEN DISPLAY INTERNATIONAL" with
   type "NHD-0420D3Z-NSW-BBW-V3". It has 4 lines with 20 characters each
   and has a blue background with white characters. The display is controlled
   with a SPI interface. More information can be found in the datasheet.

   Datasheet: https://www.mouser.com/ds/2/291/NHD-0420D3Z-NSW-BBW-V3-11396.pdf
*/

/*
   The class "LCD" for the 4 lines x 20 characters SPI LCD.
*/
class LCD {
  private:
    const byte SS_pin;  //Slave select

    const byte BRIGHTNESS = 8;  //Brightness level 1-8
    const byte CONTRAST   = 40; //Contrast level 1-50

  public:
    LCD(byte _SS_pin);                                      //Constructor
    void init();                                            //Initialise display
    void SPI_settings();                                    //Configure SPI
    void setBrightness(byte brightness);                    //Set brightness
    void setContrast(byte contrast);                        //Set contrast
    void placeCursor(byte row, byte col);                   //Place cursor
    void writeText(char text[]);                            //Write text
    void writeTextCursor(byte row, byte col, char text[]);  //Write text at cursor
    void reset();                                           //Reset
    void wait(long x);                                      //Delay
};

/*
   The "LCD" class constructor.
*/
LCD::LCD(byte _SS_pin): SS_pin(_SS_pin) {
  init();
}

/*
   Initialisation of the LCD display.
*/
void LCD::init() {
  pinMode(SS_pin, OUTPUT);
  digitalWrite(SS_pin, HIGH);
  reset();
  setBrightness(BRIGHTNESS);
  setContrast(CONTRAST);
}

/*
   Configuration of SPI.
*/
void LCD::SPI_settings() {
  //https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));
}

/*
   This method sets the brightness of the display.
   The brightness levels are 1-8.
*/
void LCD::setBrightness(byte brightness) {
  SPI.begin();
  SPI_settings();
  digitalWrite(SS_pin, LOW); wait(10);
  SPI.transfer(0xFE);        wait(420);
  SPI.transfer(0x53);        wait(420);
  SPI.transfer(brightness);  wait(420);
  digitalWrite(SS_pin, HIGH);
  SPI.endTransaction();
}

/*
   This method sets the contrast of the display.
   The contrast levels are 1-50.
*/
void LCD::setContrast(byte contrast) {
  SPI.begin();
  SPI_settings();
  digitalWrite(SS_pin, LOW);  wait(10);
  SPI.transfer(0xFE);         wait(420);
  SPI.transfer(0x52);         wait(420);
  SPI.transfer(contrast);     wait(420);
  digitalWrite(SS_pin, HIGH);
  SPI.endTransaction();
}

/*
   This method places the cursor at a specific row and column.
   The rows are 1-4 and the columns are 1-20.
*/
void LCD::placeCursor(byte row, byte col) {
  byte loc;

  switch (row) {
    case 1:  loc = 0x00 + (col - 1); break;
    case 2:  loc = 0x40 + (col - 1); break;
    case 3:  loc = 0x14 + (col - 1); break;
    case 4:  loc = 0x54 + (col - 1); break;
    default: loc = 0x00 + (col - 1); break;
  }

  SPI.begin();
  SPI_settings();
  digitalWrite(SS_pin, LOW);
  SPI.transfer(0xFE);   wait(420);
  SPI.transfer(0x45);   wait(420);
  SPI.transfer(loc);    wait(420);
  digitalWrite(SS_pin, HIGH);
  SPI.endTransaction();
}

/**
   This method writes text to the display.
   The text is a char array e.g. "Hello world!".
*/
void LCD::writeText(char text[]) {
  SPI.begin();
  SPI_settings();  wait(40);
  digitalWrite(SS_pin, LOW);

  for (byte i = 0; i < strlen(text); i++) {
    SPI.transfer(text[i]);   wait(95);
  }

  digitalWrite(SS_pin, HIGH);
  SPI.endTransaction();
}

/*
   This method combines the placement of the cursor
   and the writing of text.
   The rows are 1-4 and the columns are 1-20.
   The text is a char array e.g. "Hello world!".
*/
void LCD::writeTextCursor(byte row, byte col, char text[]) {
  placeCursor(row, col);
  writeText(text);
}

/*
   This method clears the display.
*/
void LCD::reset() {
  SPI.begin();
  SPI_settings();
  digitalWrite(SS_pin, LOW);
  wait(15);
  SPI.transfer(0xFE); wait(420);
  SPI.transfer(0x51); wait(420);
  digitalWrite(SS_pin, HIGH);
  SPI.endTransaction();
}

/*
   This method functions as a delay.
   The parameter x is the amount of nops.
*/
void LCD::wait(long x) {
  for (long i = 0; i < x; i++) {
    asm ("nop"); //http://home.roboticlab.eu/en/examples/timer/delay
  }
}

#endif
