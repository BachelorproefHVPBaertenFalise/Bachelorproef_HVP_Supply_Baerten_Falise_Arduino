#include "Arduino.h"
#include <SPI.h>

#ifndef DAC_12bit_H
#define DAC_12bit_H

/* Authors: Sven Baerten (sven.baerten@student.uhasselt.be) & 
            Wouter Falise (wouter.falise@student.uhasselt.be)
   
   Library for the 12-bit DAC "MPC4922" made by "Microchip".
   There are 2 output ports PortA and PortB. 12-bit means
   that there are 4096 levels or 0-4095.
   More information can be found in the datasheet.

   Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/21897a.pdf
*/

/*
    The class "DAC_12bit" for the "MPC4922".
*/
class DAC_12bit {
  private:
    byte pin_SS;    //Slave select

  public:
    DAC_12bit(byte _pin_SS);      //Constructor
    void init();                  //Initialise DAC
    void writePortA(int value_A); //Write voltage PortA
    void writePortB(int value_B); //Write voltage PortB
    void reset();                 //Reset DAC
};

/*
   The "DAC_12bit" class constructor.
*/
DAC_12bit::DAC_12bit(byte _SS_pin): pin_SS(_SS_pin) {
  init();
}

/*
   Initialisation of the DAC.
*/
void DAC_12bit::init() {
  pinMode(pin_SS, OUTPUT);
  digitalWrite(pin_SS, HIGH);
  reset();
}

/*
   This method writes a voltage level to the output
   PortA of the 12-bit DAC. The value is in the range 0-4095.
*/
void DAC_12bit::writePortA(int value_A) {
  //Datasheet pg 18 5.0
  //Register 5-1 and figure 5-1 show the write command.

  unsigned int writeCommand_PortA = 0b0111000000000000 | (value_A & 0b111111111111);
  //The command is a 16-bit word. The 4 MSB '0111' are configuration bits.
  //The 12 LSB represent the value written to PortA. This value is
  //between 0-4095.
  //The operation 0b0111000000000000 | (value_A & 0b111111111111) combines the
  //configuration bits and the value.

  SPI.begin();
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));  //https://www.arduino.cc/en/Reference/SPI
  digitalWrite(pin_SS, LOW);
  SPI.transfer16(writeCommand_PortA);
  digitalWrite(pin_SS, HIGH);
  SPI.endTransaction();
}

/*
   This method writes a voltage level to the output
   PortB of the 12-bit DAC. The value is in the range 0-4095.
*/
void DAC_12bit::writePortB(int value_B) {
  //Same principle as PortA. 4 MSB are now '1111'.
  unsigned int writeCommand_PortB = 0b1111000000000000 | (value_B & 0b111111111111);

  SPI.begin();
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));
  digitalWrite(pin_SS, LOW);
  SPI.transfer16(writeCommand_PortB);
  digitalWrite(pin_SS, HIGH);
  SPI.endTransaction();
}

/*
   This method resets the 12-bit DAC
   and writes 0 V to both PortA and PortB.
*/
void DAC_12bit::reset() {
  writePortA(0);
  writePortB(0);
}

#endif
