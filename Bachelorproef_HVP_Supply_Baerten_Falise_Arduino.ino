/*
   Bachelor's thesis: Design and assembly of a control circuit with microcontroller and user interface for OEM high voltage modules
   Students: Sven Baerten (sven.baerten@student.uhasselt.be) & Wouter Falise (wouter.falise@student.uhasselt.be)
   Discipline: FIIW 3BA EA-ICT
   Mentors: prof. dr. ir. Michael Daenen & ing. Jorne Carolus
   Year: 2017-2018
*/

/*
   This Arduino sketch is made for the Arduino Nano inside the high voltage power supply.
*/

#include "ADC_12bit.h"
#include "DAC_12bit.h"
#include "GPIO_expander.h"
#include "LCD.h"
#include "Numpad.h"

/*
   Finite-state machine (FSM)
*/
enum STATE {
  MAIN,                     //Selection: Standalone or USB
  STANDALONE,               //Mode: "Constant voltage" or "Constant Current"
  USB,                      //USB
  CONSTANT_VOLTAGE,         //Power supply used as a constant voltage supply
  CONSTANT_CURRENT,         //Power supply used as a constant current supply
  EMERGENCY_STOP,           //EMERGENCY STOP
};

STATE state = MAIN;
STATE temp_state;     //Variable to temporary store the current state
/********************/

/*
   Variables for the MPS high voltage modules: datasheet https://www.spellmanhv.com/en/Products/MPS
      Type MPS2N10/24: CH1 = 0...-2000V
      Type MPS2P10/24: CH2 = 0...+2000V
*/
//U = voltage, I = current, soll = desired value, ist = measured value, DAC = set value for DAC
int   CH1_U_SOLL = 0;
float CH1_I_SOLL = 0;
int   CH2_U_SOLL = 0;
float CH2_I_SOLL = 0;
int   CH1_U_IST  = 0;
float CH1_I_IST  = 0;
int   CH2_U_IST  = 0;
float CH2_I_IST  = 0;
int   CH1_U_DAC  = 0;
int   CH2_U_DAC  = 0;

int CH1_U_LIMIT = 2000; //Voltage limit is initially 2000 V
int CH2_U_LIMIT = 2000;
float CH1_I_LIMIT = 5;  //Current limit is initially 5 mA
float CH2_I_LIMIT = 5;

const int U = 2000;           //Maximum voltage of the high voltage module is 2000 V
const float I = 5.000;        //Maximum current of the high voltage module is 5 mA

const float correction = 1.18;  //Tuning of high voltage module monitors

//Regulator
const int voltage_offset_CH1 = 0;   //Calibrating voltage of CH1/2
const int voltage_offset_CH2 = 0;

//Arduino pins to control the "enable pin" to turn the high voltage module of CH1/2 on or off
const byte pin_CH1_enable = 6;
const byte pin_CH2_enable = 5;

//True = CH1/2 active
bool CH1_ON = false;
bool CH2_ON = false;

//Variables used to display more screens on the LCD e.g. in "Constant voltage" there is a screen for the voltages and one for the currents
bool CONSTANT_VOLTAGE_toggle_window = true;
bool CONSTANT_CURRENT_toggle_window = true;

const byte pin_emergency_stop = A7;  //Arduino pin connected to the emergency stop, detects if emergency stop is active
const byte pin_lock = A6;            //Arduino pin connected to the lock, detects if the power supply is locked

bool emergency_stop = false;
/********************/

/*
   Variables for the GPIO expander "MCP23S17": datasheet http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf
      Inputs:  Buttons and rotary encoders for user interface
      Outputs: LEDs for user interface
*/
//3 buttons connected to expander: On/off button for CH1, on/off button for CH2 and button for step size selection
//4 rotary encoders connected to expander: 2 for voltage and current of CH1 and 2 for CH2

const byte pin_SS_GPIO_expander = 9;                                //Arduino pin slave select of the GPIO expander
GPIO_expander gpio_expander = GPIO_expander(pin_SS_GPIO_expander);  //Instantiate GPIO expander

const byte pin_GPIO_expander_interrupt = 2;   //Arduino pin connected to the GPIO expander interrupt pin, changes on the expander inputs cause a voltage drop

byte rot_enc_step_size = 100;  //Step size of the rotary encoders: 3 options -> x1, x10 of x100 (default)
const float factor_I = 0.01;  //Multiplication factor for the step size of the current I

//True = on
bool CH1_LED = false;           //LED: CH1 on/off
bool CH2_LED = false;           //LED: CH2 on/off
bool step_size_LED_x1 = false;  //LED: Step size x1   on/off
bool step_size_LED_x10 = false; //LED: Step size x10  on/off
bool step_size_LED_x100 = true; //LED: Step size x100 on/off (default ON)
/********************/

/*
   Variables for the numpad: 3x4 or 3 columns and 4 rows matrix
   Datasheet: http://www.farnell.com/datasheets/1662617.pdf
*/
//Arduino pins connected to the rows and columns
const byte pin_numpad_row1  = A0;
const byte pin_numpad_row2  = A1;
const byte pin_numpad_row3  = A2;
const byte pin_numpad_row4  = A3;
const byte pin_numpad_col1  = A4;
const byte pin_numpad_col2  = A5;
const byte pin_numpad_col3  = 4;

//Instantiate numpad
Numpad numpad = Numpad(pin_numpad_row1, pin_numpad_row2, pin_numpad_row3, pin_numpad_row4,
                       pin_numpad_col1, pin_numpad_col2, pin_numpad_col3);
/********************/

/*
   Variables for the LCD display "NHD-0420D3Z-NSW-BBW-V3"
   Datasheet: https://www.mouser.com/ds/2/291/NHD-0420D3Z-NSW-BBW-V3-11396.pdf
*/
const byte pin_SS_display = 10;    //Arduino pin slave select of the LCD
LCD lcd = LCD (pin_SS_display);    //Instantiate LCD

bool LCD_writeTemplate = true;     //Variable used to determine whether an LCD template can be written

unsigned long LCD_previousUpdate = 0;  //Time of previouse LCD refresh (in ms)
int LCD_refreshRate = 200;             //LCD refresh rate (in ms)
/********************/

/*
   Variables for the:
      12-bit ADC or analog-to-digital converter "MPC3204":
        Reading the voltage and current monitor from the high voltage modules
        Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/21298c.pdf
      12-bit DAC or digital-to-analog converter "MPC4922":
        Programming the high voltage modules
        Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/21897a.pdf
*/
const byte pin_SS_DAC = 8;  //Arduino pin slave select of the DAC
DAC_12bit dac = DAC_12bit(pin_SS_DAC);

const byte pin_SS_ADC = 7;  //Arduino pin slave select of the ADC
ADC_12bit adc = ADC_12bit(pin_SS_ADC);

const int resolution = 4095;  //ADC and DAC are 12 bit, 2^12 = 4096 or 0...4095
/********************/

/*
   Variables for the USB communication
*/
bool USB_mode = false;        //True = USB mode active
bool USB_connection = false;  //True = connection established

char USB_command[31];              //USB command sent by the computer
byte USB_command_length = 0;       //Number of characters in the USB command
bool USB_command_received = false; //True = USB command received

char USB_opcode[22];              //The opcode of the USB command
char USB_parameter[8];            //The parameter of the USB command

//The USB connection times out when millis() - USB_watchdog_timer > USB_watchdog_timeout
unsigned long USB_watchdog_timer = 0;       //in ms
unsigned long USB_watchdog_timeout = 12000; //in ms
/********************/


/*
   Program setup.
*/
void setup() {
  Serial.begin(38400);

  pinMode(pin_emergency_stop, INPUT);
  pinMode(pin_lock, INPUT);
  pinMode(pin_GPIO_expander_interrupt, INPUT);
  pinMode(pin_CH1_enable, OUTPUT);
  pinMode(pin_CH2_enable, OUTPUT);

  lcd.init();
  gpio_expander.init();
  adc.init();
  dac.init();

  CH1_turnOFF();
  CH2_turnOFF();
}

/*
   Program loop.
*/
void loop() {
  char key = numpad.getKeyPressed();  //Checks whether a numpad key is pressed
  read_emergency_stop();              //Checks whether the emergency stop is active

  //FSM
  switch (state) {
    case MAIN:
      LCD_update();

      if (key == '*') {           //STANDALONE
        state = STANDALONE;       //Load STANDALONE state
        LCD_writeTemplate = true; //Draw LCD template of STANDALONE state
      } else if (key == '#') {    //USB
        state = USB;              //Load USB state
        LCD_writeTemplate = true; //Draw LCD template of USB state
        while (Serial.available()) Serial.read(); //Clears the serial buffer
      }

      break;

    case STANDALONE:
      LCD_update();

      gpio_expander.clearInterrupt(); //Prevent reading the gpio expander until the next state

      if (key == '*') {           //Constant voltage
        state = CONSTANT_VOLTAGE;
        LCD_writeTemplate = true;
      } else if (key == '#') {
        state = CONSTANT_CURRENT; //CONSTANT CURRENT
        LCD_writeTemplate = true;
      } else if (key == '0') {    //Back to MAIN
        state = MAIN;
        LCD_writeTemplate = true;
      }

      break;

    case USB:
      LCD_update();

      USB_mode = true;
      CH1_turnOFF();       //Turn voltage module off
      CH2_turnOFF();
      USB_communication(); //Read USB input to verify a connection

      if (key == '*') { //Back to MAIN
        state = MAIN;
        LCD_writeTemplate = true;
        reset();       //Reset all settings
      }

      break;

    case EMERGENCY_STOP:
      LCD_update();

      CH1_turnOFF();
      CH2_turnOFF();
      gpio_expander.clearInterrupt();

      if(USB_mode) USB_communication();

      break;

    case CONSTANT_VOLTAGE:
      LCD_update();

      //The CONSTANT_VOLTAGE LCD screen can be toggled between:
      //Template 1: Shows soll and ist voltages
      //Template 2: Shows ist currents
      if (CONSTANT_VOLTAGE_toggle_window == true) { //Template 1 is shown
        if (key == '*' && USB_mode == false) {     //Back to STANDALONE
          LCD_writeTemplate = true;
          state = STANDALONE;
          reset();
        } else if (key == '#') {
          CONSTANT_VOLTAGE_toggle_window = false;  //Show template 2
          LCD_writeTemplate = true;
        }
      } else {                                    //Template 2 is shown
        if (key == '#') {
          CONSTANT_VOLTAGE_toggle_window = true;   //Show template 1
          LCD_writeTemplate = true;
        }
      }

      if (USB_mode == true) USB_communication(); //USB commands are read if USB mode is active
      else if (digitalRead(pin_GPIO_expander_interrupt) == LOW) interrupt_GPIO_expander();
      //Interrupt pin of the GPIO expander goes LOW when an input changes value

      if (CH1_ON == true) { //CH1 is ON
        CH1_U_IST = ADC_getVoltage_CH1(); //Read the voltage off CH1
        CH1_I_IST = ADC_getCurrent_CH1(); //Read the current off CH1

        //Regulator
        //U_DAC is the voltage written to the DAC and U_SOLL is the desired voltage
        //The goal is an output voltage that climbs to the SOLL value
        if (CH1_I_IST > CH1_I_LIMIT) {
          CH1_U_DAC--;
        } else {
          if (CH1_U_DAC == CH1_U_SOLL + voltage_offset_CH1) ;
          else if (CH1_U_DAC < CH1_U_SOLL + voltage_offset_CH1) CH1_U_DAC++;
          else if (CH1_U_DAC > CH1_U_SOLL + voltage_offset_CH1) CH1_U_DAC--;
        }
        if (CH1_U_DAC > 2000) CH1_U_DAC = 2000;
        else if (CH1_U_SOLL == 0) CH1_U_DAC = 0;
        DAC_writeVoltage_CH1(CH1_U_DAC); //Write CH1_U_DAC to DAC
      }

      if (CH2_ON == true) { //CH2 is ON
        CH2_U_IST = ADC_getVoltage_CH2(); //Read the voltage off CH2
        CH2_I_IST = ADC_getCurrent_CH2(); //Read the current off CH2

        //Regulator
        if (CH2_I_IST > CH2_I_LIMIT) {
          CH2_U_DAC--;
        } else {
          if (CH2_U_DAC == CH2_U_SOLL + voltage_offset_CH2) ;
          else if (CH2_U_DAC < CH2_U_SOLL + voltage_offset_CH2) CH2_U_DAC++;
          else if (CH2_U_DAC > CH2_U_SOLL + voltage_offset_CH2) CH2_U_DAC--;
        }
        if (CH2_U_DAC > 2000) CH2_U_DAC = 2000;
        else if (CH2_U_SOLL == 0) CH2_U_DAC = 0;
        DAC_writeVoltage_CH2(CH2_U_DAC);
      }

      break;

    case CONSTANT_CURRENT:
      LCD_update();

      //The CONSTANT_CURRENT LCD screen can be toggled between:
      //Template 1: Shows soll and ist currents
      //Template 2: Shows ist voltages
      if (CONSTANT_CURRENT_toggle_window == true) { //Template 1 is shown
        if (key == '*' && USB_mode == false) {      //Back to STANDALONE
          LCD_writeTemplate = true;
          state = STANDALONE;
          reset();
        } else if (key == '#') {
          CONSTANT_CURRENT_toggle_window = false;   //Show template 2
          LCD_writeTemplate = true;
        }
      } else {                                      //Template 2 is shown
        if (key == '#') {
          CONSTANT_CURRENT_toggle_window = true;    //Show Template 1
          LCD_writeTemplate = true;
        }
      }

      if (USB_mode == true) USB_communication();
      else if (digitalRead(pin_GPIO_expander_interrupt) == LOW) interrupt_GPIO_expander();

      if (CH1_ON == true) {
        CH1_U_IST = ADC_getVoltage_CH1();
        CH1_I_IST = ADC_getCurrent_CH1();

        //Regulator
        if (CH1_U_IST > CH1_U_LIMIT) {
          CH1_U_DAC--;
        }
        else {
          if (CH1_I_IST == CH1_I_SOLL ) ;
          else if (CH1_I_IST < CH1_I_SOLL ) CH1_U_DAC++;
          else if (CH1_I_IST > CH1_I_SOLL ) CH1_U_DAC--;
        }
        if (CH1_U_DAC > 2000) CH1_U_DAC = 2000;
        else if (CH1_I_SOLL == 0 || CH1_U_DAC < 0) CH1_U_DAC = 0;
        DAC_writeVoltage_CH1(CH1_U_DAC);
      }


      if (CH2_ON == true) {
        CH2_U_IST = ADC_getVoltage_CH2();
        CH2_I_IST = ADC_getCurrent_CH2();

        //Regulator
        if (CH2_U_IST > CH2_U_LIMIT) {
          CH2_U_DAC--;
        } else {
          if (CH2_I_IST == CH2_I_SOLL ) ;
          else if (CH2_I_IST < CH2_I_SOLL ) CH2_U_DAC++;
          else if (CH2_I_IST > CH2_I_SOLL ) CH2_U_DAC--;
        }
        if (CH2_U_DAC > 2000) CH2_U_DAC = 2000;
        else if (CH2_I_SOLL == 0 || CH2_U_DAC < 0) CH2_U_DAC = 0;
        DAC_writeVoltage_CH2(CH2_U_DAC);
      }

      break;

    default: break;
  }
}

/*
   This method processes the interrupt pin of the GPIO expander.
   The input buttons and rotary encoders are read.
*/
void interrupt_GPIO_expander() {
  //Byte of port A: GPIO_A<7:0>, all inputs for the rotary encoders
  byte GPIO_portA = gpio_expander.readPortA_interrupt();
  //Byte of port B: GPIO_B<7:0>, 3 LSB are inputs for the 3 buttons
  byte GPIO_portB = gpio_expander.readPortB_interrupt();

  //The 3 LSB of port B are the inputs of buttons and are masked
  byte GPIO_portB_masked = GPIO_portB & 0b00000111;

  static bool button_pushed = false; //Functions as a Schmitt trigger
  //A button is pressed when one of the 3 LSB is 0 e.g. 0bxxxxx110
  if ( GPIO_portB_masked != 0b00000111) {
    if (button_pushed == false) {
      button_pushed = true;
      if (read_switch_LOCK() == false) { //Not locked
        switch (GPIO_portB_masked) {
          case 0b00000110: read_CH1_button(); break;        //Button B<0>
          case 0b00000101: read_CH2_button(); break;        //Button B<1>
          case 0b00000011: read_step_size_button(); break;  //Button B<2>
          default:  break;
        }
      }
    }
  } else button_pushed = false; //No button is pressed

  //A bit of port A is 0 when a rotary encoder is turned
  if (GPIO_portA != 0b11111111 && button_pushed == false) {
    if (read_switch_LOCK() == false) { //Not locked
      if (state == CONSTANT_VOLTAGE) {
        //"CONSTANT_VOLTAGE" state: Voltage rotary encoders set voltage
        //                         and current rotary encoders set current limit
        switch (GPIO_portA) {
          //Rotary 1: CH1_U_SOLL
          case 0b11111110:  //Clockwise
            if ( CH1_U_SOLL + rot_enc_step_size > 2000) CH1_U_SOLL = 2000;
            else CH1_U_SOLL += rot_enc_step_size;
            break;
          case 0b11111100:  //Counterclockwise
            if ( CH1_U_SOLL - rot_enc_step_size < 0) CH1_U_SOLL = 0;
            else CH1_U_SOLL -= rot_enc_step_size;
            break;

          //Rotary 2: CH1_I_LIMIT
          case 0b11111011:  //Clockwise
            if ( CH1_I_LIMIT + (factor_I * rot_enc_step_size) > 5) CH1_I_LIMIT = 5;
            else CH1_I_LIMIT += (factor_I * rot_enc_step_size);
            break;
          case 0b11110011:  //Counterclockwise
            if ( CH1_I_LIMIT - (factor_I * rot_enc_step_size) < 0) CH1_I_LIMIT = 0;
            else CH1_I_LIMIT -= (factor_I * rot_enc_step_size);
            break;

          //Rotary 3: CH2_U_SOLL
          case 0b11101111:  //Clockwise
            if ( CH2_U_SOLL + rot_enc_step_size > 2000) CH2_U_SOLL = 2000;
            else CH2_U_SOLL += rot_enc_step_size;
            break;
          case 0b11001111:  //Counterclockwise
            if ( CH2_U_SOLL - rot_enc_step_size < 0) CH2_U_SOLL = 0;
            else CH2_U_SOLL -= rot_enc_step_size;
            break;

          //Rotary 4: CH2_I_LIMIT
          case 0b10111111:  //Clockwise
            if ( CH2_I_LIMIT + (factor_I * rot_enc_step_size) > 5) CH2_I_LIMIT = 5;
            else CH2_I_LIMIT += (factor_I * rot_enc_step_size);
            break;
          case 0b00111111:  //Counterclockwise
            if ( CH2_I_LIMIT - (factor_I * rot_enc_step_size) < 0) CH2_I_LIMIT = 0;
            else CH2_I_LIMIT -= (factor_I * rot_enc_step_size);
            break;

          default: break;
        }
      } else if (state == CONSTANT_CURRENT) {
        //"CONSTANT_CURRENT" state: Voltage rotary encoders set voltage limit
        //                          and current rotary encoders set current
        switch (GPIO_portA) {
          //Rotary 1: CH1_U_LIMIT
          case 0b11111110:  //Clockwise
            if ( CH1_U_LIMIT + rot_enc_step_size > 2000) CH1_U_LIMIT = 2000;
            else CH1_U_LIMIT += rot_enc_step_size;
            break;
          case 0b11111100:  //Counterclockwise
            if ( CH1_U_LIMIT - rot_enc_step_size < 0) CH1_U_LIMIT = 0;
            else CH1_U_LIMIT -= rot_enc_step_size;
            break;

          //Rotary 2: CH1_I_SOLL
          case 0b11111011:  //Clockwise
            if ( CH1_I_SOLL + (factor_I * rot_enc_step_size) > 5) CH1_I_SOLL = 5;
            else CH1_I_SOLL += (factor_I * rot_enc_step_size);
            break;
          case 0b11110011:  //Counterclockwise
            if ( CH1_I_SOLL - (factor_I * rot_enc_step_size) < 0) CH1_I_SOLL = 0;
            else CH1_I_SOLL -= (factor_I * rot_enc_step_size);
            break;

          //Rotary 3: CH2_U_LIMIT
          case 0b11101111:  //Clockwise
            if ( CH2_U_LIMIT + rot_enc_step_size > 2000) CH2_U_LIMIT = 2000;
            else CH2_U_LIMIT += rot_enc_step_size;
            break;
          case 0b11001111:  //Counterclockwise
            if ( CH2_U_LIMIT - rot_enc_step_size < 0) CH2_U_LIMIT = 0;
            else CH2_U_LIMIT -= rot_enc_step_size;
            break;

          //Rotary 4: CH2_I_SOLL
          case 0b10111111:  //Clockwise
            if ( CH2_I_SOLL + (factor_I * rot_enc_step_size) > 5) CH2_I_SOLL = 5;
            else CH2_I_SOLL += (factor_I * rot_enc_step_size);
            break;
          case 0b00111111:  //Counterclockwise
            if ( CH2_I_SOLL - (factor_I * rot_enc_step_size) < 0) CH2_I_SOLL = 0;
            else CH2_I_SOLL -= (factor_I * rot_enc_step_size);
            break;

          default: break;
        }
      }

    }
  }
}

/*
   This method resets the program.
*/
void reset() {
  CH1_turnOFF();
  CH2_turnOFF();

  CH1_U_SOLL = 0;
  CH1_I_SOLL = 0;
  CH2_U_SOLL = 0;
  CH2_I_SOLL = 0;
  CH1_U_IST  = 0;
  CH1_I_IST  = 0;
  CH2_U_IST  = 0;
  CH2_I_IST  = 0;
  CH1_U_DAC  = 0;
  CH2_U_DAC  = 0;
  CH1_U_LIMIT = 2000;
  CH2_U_LIMIT = 2000;
  CH1_I_LIMIT = 5;
  CH2_I_LIMIT = 5;

  LCD_writeTemplate = true;
  LCD_previousUpdate = 0;
  CONSTANT_VOLTAGE_toggle_window = true;
  CONSTANT_CURRENT_toggle_window = true;

  rot_enc_step_size = 100;
  step_size_LED_x1 = false;
  step_size_LED_x10 = false;
  step_size_LED_x100 = true;
  write_LEDs();

  USB_mode = false;
  USB_connection = false;
  USB_watchdog_timer = 0;
  clear_USB_command();

  emergency_stop = false;
}

/*
   This method updates the LCD.
*/
void LCD_update() {
  //The LCD has 4 rows with 20 characters each
  switch (state) {
    case MAIN:
      if (LCD_writeTemplate == true) { //Must the template be written
        lcd.reset(); //Clear the LCD
        lcd.writeTextCursor(1, 1, "     Selection      ");
        lcd.writeTextCursor(3, 1, "   Standalone '*'   ");
        lcd.writeTextCursor(4, 1, "   USB        '#'   ");
        LCD_writeTemplate = false;    //Template is only written once
      }
      break;

    case STANDALONE:
      if (LCD_writeTemplate == true) {
        lcd.reset();
        lcd.writeTextCursor(1, 1, "       Mode         ");
        lcd.writeTextCursor(2, 1, "Constant voltage '*'");
        lcd.writeTextCursor(3, 1, "Constant current '#'");
        lcd.writeTextCursor(4, 1, "Back '0'");
        LCD_writeTemplate = false;
      }
      break;

    case USB:
      if (LCD_writeTemplate == true) {
        lcd.reset();
        lcd.writeTextCursor(1, 1, "        USB         ");
        lcd.writeTextCursor(2, 1, "Awaiting connection ");
        lcd.writeTextCursor(4, 1, "Stop '*'            ");
        LCD_writeTemplate = false;
      }
      break;

    case EMERGENCY_STOP:
      if (LCD_writeTemplate == true) {
        lcd.reset();
        lcd.writeTextCursor(1, 1, "   EMERGENCY STOP   ");
        LCD_writeTemplate = false;
      }
      break;

    case CONSTANT_VOLTAGE:
      if (LCD_writeTemplate == true) {
        lcd.reset();
        if (CONSTANT_VOLTAGE_toggle_window == true) { //Template with voltages
          if (USB_mode == true) lcd.writeTextCursor(1, 1, "USB  OUT      SET   ");
          else lcd.writeTextCursor(1, 1, "     OUT      SET   ");
          lcd.writeTextCursor(2, 1, "CH1: -    V   -    V");
          lcd.writeTextCursor(3, 1, "CH2: +    V   +    V");
          if (USB_mode == true) lcd.writeTextCursor(4, 1, "          Current'#'"); 
          else lcd.writeTextCursor(4, 1, "Stop'*'   Current'#'");
        } else { //Template with currents
          if (USB_mode == true) lcd.writeTextCursor(1, 1, "USB  OUT     LIMIT  ");
          else lcd.writeTextCursor(1, 1, "     OUT     LIMIT  ");
          lcd.writeTextCursor(2, 1, "CH1:      mA      mA");
          lcd.writeTextCursor(3, 1, "CH2:      mA      mA");
          lcd.writeTextCursor(4, 1, "          Voltage'#'");
        }
        LCD_writeTemplate = false;
      }
      //Update voltages and currents off CH1/CH2 shown on the LCD
      if (millis() - LCD_previousUpdate >= LCD_refreshRate) { //Periodic updates
        LCD_previousUpdate = millis();
        char voltage[5];
        char current[6];
        if (CONSTANT_VOLTAGE_toggle_window == true) { //Template with voltages
          //CH1
          snprintf(voltage, 5, "%04d", CH1_U_IST); //http://www.cplusplus.com/reference/cstdio/snprintf/
          lcd.writeTextCursor(2, 7, voltage);
          snprintf(voltage, 5, "%04d", CH1_U_SOLL);
          lcd.writeTextCursor(2, 16, voltage);
          //CH2
          snprintf(voltage, 5, "%04d", CH2_U_IST);
          lcd.writeTextCursor(3, 7, voltage);
          snprintf(voltage, 5, "%04d", CH2_U_SOLL);
          lcd.writeTextCursor(3, 16, voltage);
        } else {                                   //Template with currents
          //CH1
          dtostrf(CH1_I_IST, 5, 3, current);       //https://dereenigne.org/arduino/arduino-float-to-string/
          lcd.writeTextCursor(2, 6, current);
          dtostrf(CH1_I_LIMIT, 5, 3, current);
          lcd.writeTextCursor(2, 14, current);
          //CH2
          dtostrf(CH2_I_IST, 5, 3, current);
          lcd.writeTextCursor(3, 6, current);
          dtostrf(CH2_I_LIMIT, 5, 3, current);
          lcd.writeTextCursor(3, 14, current);
        }
      }
      break;

    case CONSTANT_CURRENT:
      if (LCD_writeTemplate == true) {
        lcd.reset();
        if (CONSTANT_CURRENT_toggle_window == true) { //Template with currents
          if (USB_mode == true) lcd.writeTextCursor(1, 1, "USB  OUT     SET    ");
          else lcd.writeTextCursor(1, 1, "     OUT     SET    ");
          lcd.writeTextCursor(2, 1, "CH1:      mA      mA");
          lcd.writeTextCursor(3, 1, "CH2:      mA      mA");
          if (USB_mode == true) lcd.writeTextCursor(4, 1, "          Voltage'#'"); 
          else lcd.writeTextCursor(4, 1, "Stop'*'   Voltage'#'");
        } else {  //Template with voltages
          if (USB_mode == true) lcd.writeTextCursor(1, 1, "USB  OUT      LIMIT ");
          else lcd.writeTextCursor(1, 1, "     OUT      LIMIT ");
          lcd.writeTextCursor(2, 1, "CH1: -    V   -    V");
          lcd.writeTextCursor(3, 1, "CH2: +    V   +    V");
          lcd.writeTextCursor(4, 1, "          Current'#'");
        }
        LCD_writeTemplate = false;
      }
      //Update currents and voltages off CH1/CH2 shown on the LCD
      if (millis() - LCD_previousUpdate >= LCD_refreshRate) {
        LCD_previousUpdate = millis();
        char voltage[5];
        char current[6];
        //Scherm CH1 updaten
        if (CONSTANT_CURRENT_toggle_window == true) {
          //CH1
          dtostrf(CH1_I_IST, 5, 3, current);
          lcd.writeTextCursor(2, 6, current);
          dtostrf(CH1_I_SOLL, 5, 3, current);
          lcd.writeTextCursor(2, 14, current);
          //CH2
          dtostrf(CH2_I_IST, 5, 3, current);
          lcd.writeTextCursor(3, 6, current);
          dtostrf(CH2_I_SOLL, 5, 3, current);
          lcd.writeTextCursor(3, 14, current);
        } else {
          //CH1
          snprintf(voltage, 5, "%04d", CH1_U_IST);
          lcd.writeTextCursor(2, 7, voltage);
          snprintf(voltage, 5, "%04d", CH1_U_LIMIT);
          lcd.writeTextCursor(2, 16, voltage);
          //CH2
          snprintf(voltage, 5, "%04d", CH2_U_IST);
          lcd.writeTextCursor(3, 7, voltage);
          snprintf(voltage, 5, "%04d", CH2_U_LIMIT);
          lcd.writeTextCursor(3, 16, voltage);
        }
      }
      break;

    default:
      break;
  }
}


/*
   This method returns the voltage of the high voltage module CH1.
*/
int ADC_getVoltage_CH1() {
  int digital = adc.getValueChannel(3);
  int analog =  ((double)digital / resolution)  * U * correction;
  return analog;
}

/*
   This method returns the voltage of the high voltage module CH2.
*/
int ADC_getVoltage_CH2() {
  int digital = adc.getValueChannel(1);
  int analog = ((double)digital / resolution)  * U * correction;
  return analog;
}

/*
   This method returns the current of the high voltage module CH1.
*/
float ADC_getCurrent_CH1() {
  int digital = adc.getValueChannel(4);
  float analog = ((double)digital / resolution ) * I * correction;
  return analog;
}

/*
   This method returns the current of the high voltage module CH2.
*/
float ADC_getCurrent_CH2() {
  int digital = adc.getValueChannel(2);
  float analog = ((double)digital / resolution) * I * correction;
  return analog;
}

/*
   This method writes a voltage to the high voltage module CH1.
*/
void DAC_writeVoltage_CH1(int voltage) {
  int dig = ((double)voltage / U) * resolution ;
  dac.writePortA(dig);
}

/*
   This method writes a voltage to the high voltage module CH2.
*/
void DAC_writeVoltage_CH2(int voltage) {
  int dig = ((double)voltage / U) * resolution;
  dac.writePortB(dig);
}

/*
   This method updates the LEDs on the user interface.
*/
void write_LEDs() {
  byte LEDs_portB = 0; //GPIO_B<7:0>
  //A high bit means the LED is on
  bitWrite(LEDs_portB, 3, CH1_LED);    //https://www.arduino.cc/reference/en/language/functions/bits-and-bytes/bitwrite/
  bitWrite(LEDs_portB, 4, CH2_LED);
  bitWrite(LEDs_portB, 5, step_size_LED_x1);
  bitWrite(LEDs_portB, 6, step_size_LED_x10);
  bitWrite(LEDs_portB, 7, step_size_LED_x100);
  gpio_expander.writePortB(LEDs_portB);  //Write to port B of the expander
}

/*
   This method turns the high voltage module CH1 on.
*/
void CH1_turnON() {
  digitalWrite(pin_CH1_enable, LOW);  //The enable pin of the module is active LOW.
  CH1_ON = true;                      //CH1 on
  CH1_LED = true;                     //LED on
  write_LEDs();                       //Update LEDs
}

/*
   This method turns the high voltage module CH1 off.
*/
void CH1_turnOFF() {
  digitalWrite(pin_CH1_enable, HIGH); //The enable pin of the module is active LOW.
  CH1_ON = false;                     //CH1 off
  CH1_LED = false;                    //LED off
  //Reset measured values
  CH1_U_IST = 0;
  CH1_I_IST = 0;
  CH1_U_DAC = 0;
  DAC_writeVoltage_CH1(0);  //0 V to output of DAC
  write_LEDs();             //Update LEDs
}

/*
   This method turns the high voltage module CH2 on.
*/
void CH2_turnON() {
  digitalWrite(pin_CH2_enable, LOW);
  CH2_ON = true;
  CH2_LED = true;
  write_LEDs();
}

/*
   This method turns the high voltage module CH2 off.
*/
void CH2_turnOFF() {
  digitalWrite(pin_CH2_enable, HIGH);
  CH2_ON = false;
  CH2_LED = false;
  CH2_U_IST = 0;
  CH2_I_IST = 0;
  CH2_U_DAC = 0;
  DAC_writeVoltage_CH2(0);
  write_LEDs();
}

/*
   This method processes the CH1 power button press.
*/
void read_CH1_button() {
  if (CH1_ON == false) CH1_turnON(); //CH1 was off and the power button is pressed
  else CH1_turnOFF();
}

/*
   This method processes the CH2 power button press.
*/
void read_CH2_button() {
  if (CH2_ON == false) CH2_turnON();
  else CH2_turnOFF();
}

/*
   This method processes the button that selects the step size for
   the rotary encoders.
*/
void read_step_size_button() {
  if (rot_enc_step_size == 1) {
    rot_enc_step_size = 10;
    step_size_LED_x1 = false;
    step_size_LED_x10 = true;
    step_size_LED_x100 = false;
  } else if (rot_enc_step_size == 10) {
    rot_enc_step_size = 100;
    step_size_LED_x1 = false;
    step_size_LED_x10 = false;
    step_size_LED_x100 = true;
  } else {
    rot_enc_step_size = 1;
    step_size_LED_x1 = true;
    step_size_LED_x10 = false;
    step_size_LED_x100 = false;
  }
  write_LEDs(); //Update LEDs
}

/*
   This method controls the emergency stop.
*/
void read_emergency_stop() {
  if (analogRead(pin_emergency_stop) < 100) {//Emergency stop active
    if (state != EMERGENCY_STOP) {           //State different then EMERGENCY_STOP
      temp_state = state;                    //Store current state so we can return later
      state = EMERGENCY_STOP;                //New state is EMERGENCY_STOP
      LCD_writeTemplate = true;              //Update LCD template
      emergency_stop = true;
    }
  } else if (state == EMERGENCY_STOP) {      //Emergency stop inactive and state is EMERGENCY_STOP
    state = temp_state;                      //Return to previous state
    LCD_writeTemplate = true;
    emergency_stop = false;
  }
}

/*
   This method returns true when the LOCK switch is enabled.
   The LOCK switch is used to lock tthe power supply.
*/
bool read_switch_LOCK() {
  return (analogRead(pin_lock) > 100);
}

/*
   This method handles the USB communication between the Arduino and the
   computer. The USB commands are made of 2 sections opcode/parameter
   e.g. "CH1_U_SOLL/2000\n". The '\n' is necessary for the Arduino as an
   indicator that the whole command is received.
*/
void USB_communication() {
  if (Serial.available() > 0) {                  //Checks whether serial data is sent
    char c = Serial.read();
    if ( (c == '\r') || (c == '\n') ) {          //Character that indicates the end of a command
      USB_command[USB_command_length] = '\0';
      USB_command_received = true;               //Complete USB command received
    }
    else USB_command[USB_command_length++] = c;  //Add character to USB command
  }

  //Watchdog timer: The connection is lost when the timer exceeds the timeout
  if ( (millis() - USB_watchdog_timer > USB_watchdog_timeout) && USB_connection == true) {
    state = USB;
    LCD_writeTemplate = true;
    CH1_turnOFF();
    CH2_turnOFF();
    USB_connection = false;
    USB_watchdog_timer = 0;
    USB_command_received = false;
    clear_USB_command();
  }

  //Process USB command when received
  if (USB_command_received == true) {
    process_USB_command();
    //USB command is made of 2 sections: e.g. "CH1_U_SOLL/1000\n" with opcode = CH1_U_SOLL and parameter = 1000

    //SYN command is used to maintain the connection between the Arduino and the computer
    //SYN/CV: CV means "Constant voltage" mode
    //SYN/CC: CC means "Constant Current" mode
    if ( (strcmp(USB_opcode, "SYN") == 0) ) {
      //strcmp: https://www.tutorialspoint.com/c_standard_library/c_function_strcmp.htm
      if ( (strcmp(USB_parameter, "CV") == 0) ) { // SYN/CV "Constant voltage" mode
        if (state != CONSTANT_VOLTAGE) {
          CH1_turnOFF();
          CH2_turnOFF();
          state = CONSTANT_VOLTAGE;
          LCD_writeTemplate = true;
        }

        if(emergency_stop) Serial.println("ACK/ES"); //ES = emergency stop
        else Serial.println("ACK/OK");               //OK means active         
        
        USB_connection = true;
        USB_watchdog_timer = millis();               //Update the watchdog timer
      }
      else if ( (strcmp(USB_parameter, "CC") == 0) ) { // SYN/CC "Constant Current" mode
        if (state != CONSTANT_CURRENT) {
          CH1_turnOFF();
          CH2_turnOFF();
          state = CONSTANT_CURRENT;
          LCD_writeTemplate = true;
        }
        
        if(emergency_stop) Serial.println("ACK/ES");
        else Serial.println("ACK/OK");
        
        USB_connection = true;
        USB_watchdog_timer = millis();     //Update the watchdog timer
      }
    }

    if (USB_connection == true && emergency_stop == false) {  
      //USB connection is established, only executes when there is no emergency stop       
      if ( (strcmp(USB_opcode, "STOP") == 0) ) {
        state = MAIN;
        reset();        
      }
      if ( (strcmp(USB_opcode, "CH1_ON") == 0) ) {
        CH1_turnON();
      }
      if ( (strcmp(USB_opcode, "CH1_OFF") == 0) ) {
        CH1_turnOFF();
      }
      if ( (strcmp(USB_opcode, "CH2_ON") == 0) ) {
        CH2_turnON();
      }
      if ( (strcmp(USB_opcode, "CH2_OFF") == 0) ) {
        CH2_turnOFF();
      }
      if ( (strcmp(USB_opcode, "CH1_U_SOLL") == 0) ) {
        if ( atoi(USB_parameter) <= U && atoi(USB_parameter) >= 0) CH1_U_SOLL = atoi(USB_parameter);
      }
      if ( (strcmp(USB_opcode, "CH1_I_SOLL") == 0) ) {
        if ( atof(USB_parameter) <= I && atof(USB_parameter) >= 0) CH1_I_SOLL = atof(USB_parameter);
      }
      if ( (strcmp(USB_opcode, "CH2_U_SOLL") == 0) ) {
        if ( atoi(USB_parameter) <= U && atoi(USB_parameter) >= 0) CH2_U_SOLL = atoi(USB_parameter);
      }
      if ( (strcmp(USB_opcode, "CH2_I_SOLL") == 0) ) {
        if ( atof(USB_parameter) <= I && atof(USB_parameter) >= 0) CH2_I_SOLL = atof(USB_parameter);
      }
      if ( (strcmp(USB_opcode, "CH1_U_LIMIT") == 0) ) {
        if ( atof(USB_parameter) <= U && atof(USB_parameter) >= 0) CH1_U_LIMIT = atof(USB_parameter);
      }
      if ( (strcmp(USB_opcode, "CH2_U_LIMIT") == 0) ) {
        if ( atof(USB_parameter) <= U && atof(USB_parameter) >= 0) CH2_U_LIMIT = atof(USB_parameter);
      }
      if ( (strcmp(USB_opcode, "CH1_I_LIMIT") == 0) ) {
        if ( atof(USB_parameter) <= I && atof(USB_parameter) >= 0) CH1_I_LIMIT = atof(USB_parameter);
     }
      if ( (strcmp(USB_opcode, "CH2_I_LIMIT") == 0) ) {
        if ( atof(USB_parameter) <= I && atof(USB_parameter) >= 0) CH2_I_LIMIT = atof(USB_parameter);
      }
      if ( (strcmp(USB_opcode, "CH1_U_IST") == 0) ) {
        Serial.println(CH1_U_IST);
      }
      if ( (strcmp(USB_opcode, "CH1_I_IST") == 0) ) {
        Serial.println(CH1_I_IST);
      }
      if ( (strcmp(USB_opcode, "CH2_U_IST") == 0) ) {
        Serial.println(CH2_U_IST);
      }
      if ( (strcmp(USB_opcode, "CH2_I_IST") == 0) ) {
        Serial.println(CH2_I_IST);
      }
    }
    clear_USB_command();
  }
}

/*
   Opcode and parameter are isolated from the USB command.
*/
void process_USB_command() {
  char *x;
  x = strtok (USB_command, "/");                //https://www.tutorialspoint.com/c_standard_library/c_function_strtok.htm
  snprintf(USB_opcode, sizeof(USB_opcode) / sizeof(USB_opcode[0]), "%s", x);
  x = strtok (NULL, "/");
  snprintf(USB_parameter, sizeof(USB_parameter) / sizeof(USB_parameter[0]), "%s", x);
}

/*
   Resets the USB command.
*/
void clear_USB_command() {
  int i;
  for (i = 0; i < sizeof(USB_command) / sizeof(USB_command[0]); i++) USB_command[i] = '\0';
  for (i = 0; i < sizeof(USB_opcode) / sizeof(USB_opcode[0]); i++) USB_opcode[i] = '\0';
  for (i = 0; i < sizeof(USB_parameter) / sizeof(USB_parameter[0]); i++) USB_parameter[i] = '\0';
  USB_command_length = 0;
  USB_command_received = false;
}
