#include "Arduino.h"
#include <SPI.h>

#ifndef GPIO_EXPANDER_H
#define GPIO_EXPANDER_H

/*
   Authors: Sven Baerten (sven.baerten@student.uhasselt.be) &
            Wouter Falise (wouter.falise@student.uhasselt.be)

   Library for the 16-bit I/O expander "MCP23S17" made by "Microchip".
   The expander consists of two 8-bit ports PORTA and PORTB. The expander
   uses a SPI interface. More information can be found in the datasheet.

   Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf
*/

/*
   The class "GPIO_expander" for the "MCP23S17".
*/
class GPIO_expander {
  private:
    //Slave select
    byte SS_pin;

    //The registers are explained in the datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf

    //Opcode Read/Write: p. 15 3.3.2 fig 3-5
    const byte opcode_read  = 0b01000001;
    const byte opcode_write = 0b01000000;

    //P. 17 3.5 table 3-4 shows the register addresses.
    //IOCON is configured for IOCON.BANK = 0.

    //Configuration register IOCON: p. 20-21 3.5.6
    const byte IOCON_address = 0x0A;
    const byte IOCON = 0b01101000;

    //I/O direction register IODIR: p. 18 3.5.1 ; 1/0 = pin as input/output
    const byte IODIRA_address = 0x00;
    const byte IODIRA = 0b11111111; //Port A<7:0>: inputs, rotary encoders
    const byte IODIRB_address = 0x01;
    const byte IODIRB = 0b00000111; //Port B<7:3>: outputs, leds ; Port B<2:0>: inputs, buttons

    //General purpose I/O port register GPIO: p. 23 3.5.10
    const byte GPIOA_address = 0x12;
    const byte GPIOB_address = 0x13;

    //Output latch register OLAT: p. 24 3.5.11
    const byte OLATA_address = 0x14;
    const byte OLATB_address = 0x15;

    //The interrupt pins A and B are OR'd together in IOCON and configured to be active LOW. 
    
    //Interrupt-on-change control register GPINTEN: p. 19 3.5.3  ; 1/0 = pin interrupt enable/disable
    const byte GPINTA_address = 0x04;
    const byte GPINTB_address = 0x05;
    const byte GPINTA = 0b01010101; //Interrupt for quadrature rotary encoder channel A
    const byte GPINTB = 0b00000111; //Interrupt for Port B<2:0> buttons

    //Interrupt captured value for port register INTCAP:  p. 23 3.5.9
    const byte INTCAPA_address = 0x10;
    const byte INTCAPB_address = 0x11;

  public:
    GPIO_expander(byte _SS_pin);                //Constructor
    void init();                                //Initialise GPIO expander
    void SPI_settings();                        //Configure SPI
    void writeByte(byte address, byte data);    //Write byte to register
    byte readByte(byte address);                //Read byte from register
    byte readPortA();                           //Read port A
    byte readPortB();                           //Read port B
    byte readPortA_interrupt();                 //Read port A at interrupt
    byte readPortB_interrupt();                 //Read port B at interrupt
    void writePortA(byte data);                 //Write port A
    void writePortB(byte data);                 //Write port B
    void clearInterrupt();                      //Resets the interrupt pin
};

/*
   The "GPIO_expander" class constructor.
*/
GPIO_expander::GPIO_expander(byte _SS_pin): SS_pin(_SS_pin) {
  init();
}

/*
   Initialisation of the GPIO expander.
*/
void GPIO_expander::init() {
  pinMode(SS_pin, OUTPUT);
  digitalWrite(SS_pin, HIGH);
  
  writeByte(IOCON_address, IOCON);
  writeByte(IODIRA_address, IODIRA);
  writeByte(IODIRB_address, IODIRB);
  writeByte(GPINTA_address, GPINTA);
  writeByte(GPINTB_address, GPINTB);

  clearInterrupt();
}

/*
   Configuration of SPI.
*/
void GPIO_expander::SPI_settings() {
  //https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));
}

/*
   This method writes a data byte to the address of a register.
*/
void GPIO_expander::writeByte(byte address, byte data) {
  SPI.begin();
  SPI_settings();
  digitalWrite(SS_pin, LOW);
  SPI.transfer(opcode_write);
  SPI.transfer(address);
  SPI.transfer(data);
  digitalWrite(SS_pin, HIGH);
  SPI.endTransaction();
}

/*
   This method reads a data byte from the address of a register.
*/
byte GPIO_expander::readByte(byte address) {
  SPI.begin();
  SPI_settings();
  digitalWrite(SS_pin, LOW);
  SPI.transfer(opcode_read);
  SPI.transfer(address);
  byte data = SPI.transfer(0x00);
  digitalWrite(SS_pin, HIGH);
  SPI.endTransaction();
  return data;
}

/*
   This method reads the current values of port A<7:0>
   of the GPIOA register. A digital 1 and 0 equal HIGH
   and LOW. Eg. if the LSB of data is 0 then A<0> is LOW.
*/
byte GPIO_expander::readPortA() {
  byte data = readByte(GPIOA_address);
  return data;
}

/*
   This method reads the current values of port B<7:0>
   of the GPIOB register. A digital 1 and 0 equal HIGH
   and LOW. Eg. if the LSB of data is 0 then B<0> is LOW.
*/
byte GPIO_expander::readPortB() {
  byte data = readByte(GPIOB_address);
  return data;
}

/*
   This method reads the values of port A<7:0> of the INTCAPA register.
   The returned data byte reflects the value at the moment of
   an interrupt. A digital 1 and 0 equal HIGH and LOW.
   Eg. if the LSB of data is 0 then A<0> was LOW at the moment of
   the interrupt.
*/
byte GPIO_expander::readPortA_interrupt() {
  byte data = readByte(INTCAPA_address);
  return data;
}

/*
   This method reads the values of port B<7:0> of the INTCAPB register.
   The returned data byte reflects the value at the moment of
   an interrupt. A digital 1 and 0 equal HIGH and LOW.
   Eg. if the LSB of data is 0 then B<0> was LOW at the moment of
   the interrupt.
*/
byte GPIO_expander::readPortB_interrupt() {
  byte data = readByte(INTCAPB_address);
  return data;
}

/*
   This method writes a data byte to the OLATA
   register of port A<7:0>. A digital 1 and 0 equal HIGH
   and LOW. Eg. if the LSB of data is 0 then A<0> is made LOW.
*/
void GPIO_expander::writePortA(byte data) {
  writeByte(OLATA_address, data);
}

/*
   This method writes a data byte to the OLATB
   register of port B<7:0>. A digital 1 and 0 equal HIGH
   and LOW. Eg. if the LSB of data is 0 then B<0> is made LOW.
*/
void GPIO_expander::writePortB(byte data) {
  writeByte(OLATB_address, data);
}

/*
   This method resets the interrupt pin.
*/
void GPIO_expander::clearInterrupt() {
  readPortA();
  readPortB();
}
#endif
