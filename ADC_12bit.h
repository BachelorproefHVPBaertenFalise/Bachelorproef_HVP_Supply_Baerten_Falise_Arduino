#include "Arduino.h"
#include <SPI.h>

#ifndef ADC_12bit_H
#define ADC_12bit_H

/*
   Authors: Sven Baerten (sven.baerten@student.uhasselt.be) &
            Wouter Falise (wouter.falise@student.uhasselt.be)

   Library for the 12-bit ADC "MPC3204" made by "Microchip".
   There are 4 input ports or channels. 12-bit means
   that there are 4096 levels or 0-4095.
   More information can be found in the datasheet.

   Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/21298c.pdf
*/

/*
    The class "ADC_12bit" for the "MPC3204".
*/
class ADC_12bit {
  private:
    byte SS_pin;  //Slave select

  public:
    ADC_12bit(byte _SS_pin);            //Constructor
    void init();                        //Initialise ADC
    int getValueChannel(byte channel);  //Read value of channel
};

/*
   The "ADC_12bit" class constructor.
*/
ADC_12bit::ADC_12bit(byte _SS_pin): SS_pin(_SS_pin) {
  init();
}

/*
   Initialisation of the ADC.
*/
void ADC_12bit::init() {
  pinMode(SS_pin, OUTPUT);
  digitalWrite(SS_pin, HIGH);
}

/*
   This method reads the digital value of a
   channel of the 12-bit ADC. The channels
   are 1-4. The value is between 0-4095.
*/
int ADC_12bit::getValueChannel(byte channel) {

  //Datasheet p. 18 fig. 6-2 shows the SPI communication
  //and p. 15 table 5-1 shows how to select the channel.
  //The single-ended mode is used.

  byte byte1_send = 0b00000110;
  byte byte2_send = ( (channel - 1) << 6 );

  SPI.begin(); //https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));  
  digitalWrite(SS_pin, LOW);
  byte byte1_received = SPI.transfer(byte1_send);
  byte byte2_received = SPI.transfer(byte2_send);
  byte byte3_received = SPI.transfer(0x00);
  digitalWrite(SS_pin, HIGH);
  SPI.endTransaction();

  //Fig. 6-2 shows that the digital value B<11:0> of a channel
  //a combination is of the 2nd and 3rd received byte from the ADC.
  return ( (byte2_received & 0b00001111) * 256  + byte3_received );
}

#endif
