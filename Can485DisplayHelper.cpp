#include "Can485DisplayHelper.h"

#define SPECIAL_COMMAND 254  //0xFE, Magic number for sending a special command
#define SETTING_COMMAND 0x7C //124, |, the pipe character: The command to change settings: baud, lines, width, backlight, splash, etc



void setCursorPosition(int row, int column) {

}

void clearDisplay() {

}

void writeToDisplay(char *content) {

}

void writeToDisplay(char *content, int row, int column) {
    
}

void writeToDisplay(float content, int decimalPlaces, int row, int column) {

}


void writeSpecialToDisplay(int content, int row, int column) {

}

