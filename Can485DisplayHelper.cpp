#include <AltSoftSerial.h>
#include "Can485DisplayHelper.h"

AltSoftSerial lcd;
#define SPECIAL_COMMAND 254  //0xFE, Magic number for sending a special command
#define SETTING_COMMAND 0x7C //124, |, the pipe character: The command to change settings: baud, lines, width, backlight, splash, etc

void DisplayInit() {
    lcd.begin(9600);
    delay(500); //wait for bootup
    clearDisplay();

    //set brightness (128-157)
    //lcd.write(0x7C);
    //lcd.write(157);

    //create custom chars
    byte fill1[8] = {
        0x10,
        0x10,
        0x10,
        0x10,
        0x10,
        0x10,
        0x10,
        0x10
    };
    byte fill2[8] = {
        0x18,
        0x18,
        0x18,
        0x18,
        0x18,
        0x18,
        0x18,
        0x18
    };
    byte fill3[8] = {
        0x1C,
        0x1C,
        0x1C,
        0x1C,
        0x1C,
        0x1C,
        0x1C,
        0x1C
    };
    byte fill4[8] = {
        0x1E,
        0x1E,
        0x1E,
        0x1E,
        0x1E,
        0x1E,
        0x1E,
        0x1E
    };
    byte fill5[8] = {
        0x1F,
        0x1F,
        0x1F,
        0x1F,
        0x1F,
        0x1F,
        0x1F,
        0x1F
    };
    byte fillMiddle[8] = {
        0x04,
        0x04,
        0x04,
        0x04,
        0x04,
        0x04,
        0x04,
        0x04  
    };
    
    if(true) {
        createCustomChar(0, fill1);
        createCustomChar(1, fill2);
        createCustomChar(2, fill3);
        createCustomChar(3, fill4);
        createCustomChar(4, fill5);
        createCustomChar(5, fillMiddle);
    }

}

void setCursorPosition(int row, int column) {
    lcd.write(254);
    if(row == 1) {
        lcd.write(127 + column);
    }
    else if(row == 2) {
        lcd.write(191 + column);
    }
}

void clearDisplay() {
    lcd.write(0xFE);
    lcd.write(0x01);
}

//writes the current display as the splash screen (not reliable)
void setSplash() {
    lcd.write(SETTING_COMMAND);
    lcd.write(0x09);
    delay(500); 
}

void writeToDisplay(char *content) {
    lcd.write(content);
}

void writeToDisplay(uint8_t content) {
    lcd.write(content);
}

void createCustomChar(int pos, byte data[]) {


    lcd.write(SETTING_COMMAND);
    lcd.write(0x4E);
    lcd.write((uint8_t)pos);
    for(int i = 0; i < 8; i++) {
        lcd.write(data[i]);
    }
    delay(50);
}

void printCustomChars() {
    clearDisplay();
    setCursorPosition(1,1);
}

/*
void SerLCD::createChar(byte location, byte charmap[])
{
  location &= 0x7; // we only have 8 locations 0-7
  beginTransmission();
  //Send request to create a customer character
  transmit(SETTING_COMMAND); //Put LCD into setting mode
  transmit(27 + location);
  for (int i = 0; i < 8; i++)
  {
    transmit(charmap[i]);
  } // for
  endTransmission();
  delay(50); //This takes a bit longer
}
*/
