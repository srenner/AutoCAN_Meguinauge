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

void writeToDisplay(char *content) {
    clearDisplay();
    setCursorPosition(1,1);
    lcd.write(content);
}

void writeToDisplay(char *content, int row, int column) {
    setCursorPosition(row, column);
    lcd.write(content);
}

void writeToDisplay(float content, int decimalPlaces, int row, int column) {
    setCursorPosition(row, column);
    static char outstr[15];
    dtostrf(content,4, decimalPlaces, outstr);
    lcd.write(outstr);
}

void writeToDisplay(uint8_t content) {
    lcd.write(content);
}

void writeSpecialToDisplay(int content, int row, int column) {
    setCursorPosition(row, column);
    lcd.write(content);
}

