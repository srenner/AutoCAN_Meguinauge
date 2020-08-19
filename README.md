# AutoCAN_Meguinauge

**MEG**asquirt + ard**UIN**o + g**AUGE** = Meguinauge. Pronounced "meh-gween-edge." Or however you want. 16x2 character LCD gauge for [MegaSquirt](http://megasquirt.info/). This project is based on my previous meguinauge project, which will no longer be actively developed. This newer version uses a smaller display, one button instead of two, and incorporates features from my carfuino project.

## Overview
* Provides a vehicle gauge that can display 1 or 2 different engine parameters at a time.
* Designed for a 1980s-1990s aesthetic.

## Hardware Details
* [Arduino-compatible AST-CAN485 Dev Board](https://www.sparkfun.com/products/14483)
* 16x2 LCD display with serial communication
* 1 LED
* 1 pushbutton

## External Libraries
* [AltSoftSerial](https://github.com/Atlantis-Specialist-Technologies/AltSoftSerial) - Make sure to use the version modified by AST to work with the AST-CAN485 dev board.

## User Interface
Engine parameters will be displayed on a 16x2 LCD display. Each engine parameter will be shown with a short code and a number. For example, "CLT 203.5" means the coolant temperature is 203.5 degrees. The short codes will be familiar to MegaSquirt users. A bar graph will also be shown for each engine parameter.

The pushbutton will be used to cycle through the different gauge displays and modes.

The LED is illuminated when one or more of the engine parameters goes outside of its predefined range. When all engine parameters return to their normal range, the LED will stay illuminated for a few more moments. The amount of time the LED remains illuminated depends on the amount of time a parameter was out of range.

### Default Warmup Display
Display coolant temp, runtime, and warmup graph on startup if the engine is cold. The graph range starts at the temp the engine started at, and is full at 160 degrees. This view becomes unavailable once the warmup temperature is reached.
| W | A | R | M | U | P |   | 1 | 0 | 4 |   | 1 | 2 | : | 0 | 6 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| █ | █ | █ | █ | █ | █ | █ | █ |   |   |   |   |   |   |   |   |


### Default Display
Display runtime and distance traveled on startup if the vehicle is already warmed up. If the vehicle started with the warmup display, automatically switch to this one once warm.
| R | U | N | T | I | M | E |   |   |   |   | 1 | 2 | : | 0 | 6 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| D | I | S | T | A | N | C | E |   |   | 0 | 1 | 7 |   | M | I |

### Single Gauge View
The engine parameter short code on the top left. Value on the top right. Entire second row is a bar graph.
| A | F | R |   |   |   |   |   |   |   |   |   | 1 | 4 | . | 7 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| █ | █ | █ | █ | █ | █ | █ | █ |   |   |   |   |   |   |   |   |

### Double Gauge View
Each row contains the engine parameter short code, value, and half-width bar graph.
| A | F | R |   | 1 | 3 | . | 2 | █ | █ | █ |   |   |   |   |   |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| T | G | T |   | 1 | 4 | . | 7 | █ | █ | █ | █ |   |   |   |   |

## MegaSquirt details
* Tested using MegaSquirt-3 with firmware version 1.5.0.
* Uses MegaSquirt's "Simplified Dash Broadcasting" as described in [this PDF](http://www.msextra.com/doc/pdf/Megasquirt_CAN_Broadcast.pdf).
