#include <Keypad.h>

#include "Arduino.h"

//"Keypad.h" library: Download, installation and example on website 
//https://playground.arduino.cc/Code/Keypad#Download


#ifndef NUMPAD_H
#define NUMPAD_H

/*
  Authors: Sven Baerten (sven.baerten@student.uhasselt.be) &
           Wouter Falise (wouter.falise@student.uhasselt.be)

  Information about the 3x4 keypad can be found in the datasheet.
  Datasheet: http://www.farnell.com/datasheets/1662617.pdf
*/

/*
   The class "Numpad" for the 3x4 key matrix.
*/
class Numpad {
  private:
    const static byte ROWS = 4;     //4 rows
    const static byte COLUMNS = 3;  //3 columns

    char keys[ROWS][COLUMNS] =  //Keys on the numpad
    {
      {'1', '2', '3'},
      {'4', '5', '6'},
      {'7', '8', '9'},
      {'*', '0', '#'}
    };

    byte rowPins[ROWS];       //List of GPIO pins of the Arduino connected to the rows
    byte columnPins[COLUMNS]; //List of GPIO pins of the Arduino connected to the columns

    //Instantiation of the Keypad object from the "Keypad.h" library
    Keypad keypad = Keypad( makeKeymap(keys), rowPins, columnPins, ROWS, COLUMNS ); 

  public:
    Numpad(byte pin_row1, byte pin_row2, byte pin_row3, byte pin_row4,  //Constructor
           byte pin_column1, byte pin_column2, byte pin_column3);

    char getKeyPressed(); //Returns the pressed key
};

/*
   The "Numpad" class constructor.
*/
Numpad::Numpad(byte pin_row1, byte pin_row2, byte pin_row3, byte pin_row4,
               byte pin_column1, byte pin_column2, byte pin_column3) {
  rowPins[0] = pin_row1;
  rowPins[1] = pin_row2;
  rowPins[2] = pin_row3;
  rowPins[3] = pin_row4;

  columnPins[0] = pin_column1;
  columnPins[1] = pin_column2;
  columnPins[2] = pin_column3;
}

/*
   Method that returns the pressed key.
*/
char Numpad::getKeyPressed() {
  char key = keypad.getKey();

  if (key != NO_KEY) {
    return key;
  }

  return -1;
}

#endif
