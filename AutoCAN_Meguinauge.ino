#include <Arduino.h>
#include "Can485DisplayHelper.h"
#include <ASTCanLib.h>
#include <math.h>
#include <AutoCAN.h>

/*
  CONFIGURATION NOTES
  ===================
  - setting DEBUG true outputs messages to the serial port
  - define BLOCK as whichever ASCII character you want to use in the bar graphs
  - define SYSTEM_MESSAGE_ID as a message id you want to send on your CAN bus
*/

#define DEBUG true              //print out debug messages on serial port
#define BLOCK 255               //block character to build bar graphs
#define SPACE 32                //space character for start animation

// CAN BUS OBJECTS //////////////////////////////////////////////


uint8_t canBuffer[8] = {};




#define MESSAGE_PROTOCOL  0     // CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
#define MESSAGE_LENGTH    8     // Data length: 8 bytes
#define MESSAGE_RTR       0     // rtr bit

volatile unsigned long canCount = 0;
volatile unsigned long canUnhandledCount = 0;

volatile st_cmd_t canMsg;


typedef struct {
  int16_t id;
  unsigned long counter;
  uint8_t* data;
} canData;


#define MSG_1512 0
#define MSG_1513 1
#define MSG_1514 2
#define MSG_1515 3
#define MSG_1516 4

volatile canData* allCanMessages[5];

volatile canData can1512;
volatile canData can1513;
volatile canData can1514;
volatile canData can1515;
volatile canData can1516;

uint8_t canBuffer1512[8] = {};
uint8_t canBuffer1513[8] = {};
uint8_t canBuffer1514[8] = {};
uint8_t canBuffer1515[8] = {};
uint8_t canBuffer1516[8] = {};

volatile canData canTemp;
uint8_t canBufferTemp[8] = {};



// SET UP PINS //////////////////////////////////////////////////

//This version of AltSoftSerial hard-codes the pins to 9 (rx) and 5(tx)
//This also disables PWM on 6 and 7

const byte BUTTON_PIN = 10;   // pushbutton to cycle through modes
const byte LED_ERR = 11;      // 'check engine' light
const byte LED_SHIFT = 12;    // shift light

// BUILD ENGINE VARIABLES ///////////////////////////////////////

#pragma region engine variables
struct EngineVariable
{
  char* shortLabel;
  float currentValue;
  float previousValue;
  float minimum;
  float maximum;
  byte decimalPlaces;
  unsigned long goodCount;
  unsigned long lowCount;
  unsigned long highCount;
};
const byte ENGINE_VARIABLE_COUNT = 20;
EngineVariable* allGauges[ENGINE_VARIABLE_COUNT];
EngineVariable engine_map   = {"MAP", 0.0, 0.0, 15.0, 250.0, 1, 0, 0, 0};     //manifold absolute pressure
EngineVariable engine_rpm   = {"RPM", 0.0, 0.0, 700.0, 6000.0, 0, 0, 0, 0};   //engine rpm
EngineVariable engine_clt   = {"CLT", 0.0, 0.0, 20.0, 240.0, 0, 0, 0, 0};     //coolant temp
EngineVariable engine_tps   = {"TPS", 0.0, 0.0, 0.0, 100.0, 0, 0, 0, 0};      //throttle position
EngineVariable engine_pw1   = {"PW1", 0.0, 0.0, 0.0, 20.0, 2, 0, 0, 0};       //injector pulse width bank 1
EngineVariable engine_pw2   = {"PW2", 0.0, 0.0, 0.0, 20.0, 2, 0, 0, 0};       //injector pulse width bank 2
EngineVariable engine_iat   = {"IAT", 0.0, 0.0, 40.0, 150.0, 0, 0, 0, 0};     //intake air temp aka 'mat'
EngineVariable engine_adv   = {"ADV", 0.0, 0.0, 10.0, 40.0, 1, 0, 0, 0};      //ignition advance
EngineVariable engine_tgt   = {"TGT", 0.0, 0.0, 10.0, 20.0, 1, 0, 0, 0};      //afr target
EngineVariable engine_afr   = {"AFR", 0.0, 0.0, 10.0, 20.0, 1, 0, 0, 0};      //air fuel ratio
EngineVariable engine_ego   = {"EGO", 0.0, 0.0, 70.0, 130.0, 0, 0, 0, 0};     //ego correction %
EngineVariable engine_egt   = {"EGT", 0.0, 0.0, 0.0, 2000.0, 0, 0, 0, 0};     //exhaust gas temp
EngineVariable engine_pws   = {"PWS", 0.0, 0.0, 0.0, 20.0, 2, 0, 0, 0};       //injector pulse width sequential
EngineVariable engine_bat   = {"BAT", 0.0, 0.0, 11.0, 15.0, 1, 0, 0, 0};      //battery voltage
EngineVariable engine_sr1   = {"SR1", 0.0, 0.0, 0.0, 999.0, 1, 0, 0, 0};      //generic sensor 1
EngineVariable engine_sr2   = {"SR2", 0.0, 0.0, 0.0, 999.0, 1, 0, 0, 0};      //generic sensor 2
EngineVariable engine_knk   = {"KNK", 0.0, 0.0, 0.0, 50.0, 1, 0, 0, 0};       //knock ignition retard
EngineVariable engine_vss   = {"VSS", 0.0, 0.0, 0.0, 140.0, 0, 0, 0, 0};      //vehicle speed
EngineVariable engine_tcr   = {"TCR", 0.0, 0.0, 0.0, 50.0, 1, 0, 0, 0};       //traction control ignition retard
EngineVariable engine_lct   = {"LCT", 0.0, 0.0, 0.0, 50.0, 1, 0, 0, 0};       //launch control timing
#pragma endregion

bool shiftLight = false;
bool previousShiftLight = false;

bool currentButtonValue = 1;
bool previousButtonValue = 1;
unsigned long buttonMillis = 0;

// WARMUP VARIABLES ///////////////////////////////////

const float STARTUP_CLT_VALUE = -200.0;
float startupCLT = STARTUP_CLT_VALUE;

// LOOP TIMER VARIABLES ///////////////////////////////

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long startPolling = 0;
unsigned long endPolling = 0;
unsigned int pollCount = 0;
unsigned int maxPollingDelay = 0;
unsigned int pollingDelayLimit = 10;
byte displayInterval = 100;
unsigned long lastDisplayMillis = 0;
unsigned int diagnosticInterval = 5000;
unsigned long lastDiagnosticMillis = 0;

// MODE VARIABLES ////////////////////////////////////

enum DisplayType {
  warmup,
  runtime,
  single,
  dual,
  diagnostic
};

struct Display
{
  DisplayType type;
  EngineVariable* gauge1;
  EngineVariable* gauge2;
};

const byte DISPLAY_COUNT = 16;
byte currentDisplayIndex = 0;
Display* allDisplays[DISPLAY_COUNT];
Display display_warmup    = {warmup, NULL, NULL};
Display display_runtime   = {runtime, NULL, NULL};
Display display_clt_iat   = {dual, &engine_clt, &engine_iat};
Display display_afr_tgt   = {dual, &engine_afr, &engine_tgt};
Display display_afr_map   = {dual, &engine_afr, &engine_map};
Display display_afr_ego   = {dual, &engine_afr, &engine_ego};
Display display_vss_rpm   = {dual, &engine_vss, &engine_rpm};
Display display_rpm_bat   = {dual, &engine_rpm, &engine_bat};
Display display_bat       = {single, &engine_bat, NULL};
Display display_clt       = {single, &engine_clt, NULL};
Display display_iat       = {single, &engine_iat, NULL};
Display display_afr       = {single, &engine_afr, NULL};
Display display_map       = {single, &engine_map, NULL};
Display display_adv       = {single, &engine_adv, NULL};
Display display_tps       = {single, &engine_tps, NULL};
Display display_pw1       = {single, &engine_pw1, NULL};

bool inError = false;

const byte DEBOUNCE_DELAY = 250;
const int REBOOT_DELAY = 2000;
const int SHIFT_LIGHT_FROM_REDLINE_WOT = 500;
const int SHIFT_LIGHT_FROM_REDLINE_CRUISE = 1000;

void(* resetFunc) (void) = 0; //declare killswitch function

void clearBuf(volatile uint8_t* Buffer){
  for (int i=0; i<8; i++){
    Buffer[i] = 0x00;
  }
}

ISR(CANIT_vect) {
  canCount++;

  unsigned i;   
  char save_canpage=CANPAGE;   
  
  unsigned mob = CANHPMOB; // get highest prio mob   
  CANPAGE = mob & 0xf0;   
  mob >>= 4; // -> mob number 0..15   
  //ASSERT( (CANSTMOB & ~0xa0) ==0); // allow only RX ready and DLC warning   
    
  canMsg.id.std = (CANIDT2>>5) | (CANIDT1 <<3);
  canTemp.id = (CANIDT2>>5) | (CANIDT1 <<3);
  
  register char length; 
  length = CANCDMOB & 0x0f;
  //clearBuf(canTemp.data[0]);
  for (i = 0; i <length; ++i)   
  {
    canMsg.pt_data[i] = CANMSG;
    //canTemp.data[i] = CANMSG;
  }   
  
  CANSTMOB = 0;           // reset INT reason   
  CANCDMOB = 0x80;        // re-enable RX on this channel   
  CANPAGE = save_canpage; // restore CANPAGE   

  if(true) 
  {
    switch(canTemp.id)
    {
      case 1512: //1512
        allCanMessages[MSG_1512]->counter++;
        memcpy(allCanMessages[MSG_1512]->data, canTemp.data, sizeof(canTemp.data));
        break;
      case 1513:
        allCanMessages[MSG_1513]->counter++;
        memcpy(allCanMessages[MSG_1513]->data, canTemp.data, sizeof(canTemp.data));
        break;
      case 1514:
        allCanMessages[MSG_1514]->counter++;
        memcpy(allCanMessages[MSG_1514]->data, canTemp.data, sizeof(canTemp.data));
        break;
      case 1515:
        allCanMessages[MSG_1515]->counter++;
        memcpy(allCanMessages[MSG_1515]->data, canTemp.data, sizeof(canTemp.data));
        break;
      case 1516:
        allCanMessages[MSG_1516]->counter++;
        memcpy(allCanMessages[MSG_1516]->data, canTemp.data, sizeof(canTemp.data));
        break;
      default:
        canUnhandledCount++;
        break;
    }
  }
}

void setup() {

  DisplayInit();

  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_SHIFT, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  

//

  clearBuffer(&canBufferTemp[0]);
  canTemp.data = &canBufferTemp[0];

  can1512.id = 1512;
  can1512.counter = 0;
  clearBuffer(&canBuffer1512[0]);
  can1512.data = &canBuffer1512[0];
  allCanMessages[0] = &can1512;

  can1513.id = 1513;
  can1513.counter = 0;
  clearBuffer(&canBuffer1513[0]);
  can1513.data = &canBuffer1513[0];
  allCanMessages[1] = &can1513;
  
  can1514.id = 1514;
  can1514.counter = 0;
  clearBuffer(&canBuffer1514[0]);
  can1514.data = &canBuffer1514[0];
  allCanMessages[2] = &can1514;
  
  can1515.id = 1515;
  can1515.counter = 0;
  clearBuffer(&canBuffer1515[0]);
  can1515.data = &canBuffer1515[0];
  allCanMessages[3] = &can1515;

  can1516.id = 1516;
  can1516.counter = 0;
  clearBuffer(&canBuffer1516[0]);
  can1516.data = &canBuffer1516[0];
  allCanMessages[4] = &can1516;


  #pragma region set allGauges
  allGauges[0] = &engine_map;
  allGauges[1] = &engine_rpm;
  allGauges[2] = &engine_clt;
  allGauges[3] = &engine_tps;
  allGauges[4] = &engine_pw1;
  allGauges[5] = &engine_pw2;
  allGauges[6] = &engine_iat;
  allGauges[7] = &engine_adv;
  allGauges[8] = &engine_tgt;
  allGauges[9] = &engine_afr;
  allGauges[10] = &engine_ego;
  allGauges[11] = &engine_egt;
  allGauges[12] = &engine_pws;
  allGauges[13] = &engine_bat;
  allGauges[14] = &engine_sr1;
  allGauges[15] = &engine_sr2;
  allGauges[16] = &engine_knk;
  allGauges[17] = &engine_vss;
  allGauges[18] = &engine_tcr;
  allGauges[19] = &engine_lct;
  #pragma endregion
  
  #pragma region set all displays

  allDisplays[0] = &display_warmup;
  allDisplays[1] = &display_runtime;
  allDisplays[2] = &display_clt_iat;
  allDisplays[3] = &display_afr_tgt;
  allDisplays[4] = &display_afr_map;
  allDisplays[5] = &display_afr_ego;
  allDisplays[6] = &display_vss_rpm;
  allDisplays[7] = &display_rpm_bat;
  allDisplays[8] = &display_bat;
  allDisplays[9] = &display_clt;
  allDisplays[10] = &display_iat;
  allDisplays[11] = &display_afr;
  allDisplays[12] = &display_map;
  allDisplays[13] = &display_adv;
  allDisplays[14] = &display_tps;
  allDisplays[15] = &display_pw1;

  #pragma endregion

  writeToDisplay("Wait for CAN bus");
  canInit(500000);                        // Initialise CAN port - must be before Serial.begin
  Serial.begin(1000000);

  /*
    The general interrupt enable is provided by ENIT bit and the specific interrupt enable for CAN
    timer overrun is provided by ENORVT bit.

    Bit 5 – ENRX: Enable Receive Interrupt
    – 0 - interrupt disabled.
    – 1- receive interrupt enabled.

    see also: https://www.avrfreaks.net/comment/165363#comment-165363

  */

  CANSTMOB |= (1 << RXOK);
  CANGIE |= (1 << ENRX);

  CANIE1 |= (1 << IEMOB14);
  CANIE1 |= (1 << IEMOB13);
  CANIE1 |= (1 << IEMOB12);
  CANIE1 |= (1 << IEMOB11);
  CANIE1 |= (1 << IEMOB10);
  CANIE1 |= (1 << IEMOB9);
  CANIE1 |= (1 << IEMOB8);

  CANIE2 |= (1 << IEMOB7);
  CANIE2 |= (1 << IEMOB6);
  CANIE2 |= (1 << IEMOB5);
  CANIE2 |= (1 << IEMOB4);
  CANIE2 |= (1 << IEMOB3);
  CANIE2 |= (1 << IEMOB2);
  CANIE2 |= (1 << IEMOB1);
  CANIE2 |= (1 << IEMOB0);

  CANGIE |= (1 << ENIT);


  if(DEBUG) {
    Serial.println("CAN bus initialized");
  }
  clearDisplay();
  bootAnimation();

  //clearBuffer(&canBufferTemp[0]);
  //canMsg.pt_data = &canBufferTemp[0];

  clearBuffer(&canBuffer[0]);
  canMsg.cmd      = CMD_RX_DATA;
  canMsg.pt_data  = &canBuffer[0];
  canMsg.ctrl.ide = MESSAGE_PROTOCOL; 
  canMsg.id.std   = 0;
  canMsg.id.ext   = 0;
  canMsg.dlc      = MESSAGE_LENGTH;
  canMsg.ctrl.rtr = MESSAGE_RTR;

  while(can_cmd(&canMsg) != CAN_CMD_ACCEPTED);
}

void loop() {
  
  //Serial.println(canCount);
  noInterrupts();
  Serial.print(allCanMessages[MSG_1512]->counter);
  Serial.print(",");
  Serial.print(allCanMessages[MSG_1513]->counter);
  Serial.print(",");
  Serial.print(allCanMessages[MSG_1514]->counter);
  Serial.print(",");
  Serial.print(allCanMessages[MSG_1515]->counter);
  Serial.print(",");
  Serial.println(allCanMessages[MSG_1516]->counter);

  processCanMessages();
  interrupts();









  previousMillis = currentMillis;
  currentMillis = millis();

  previousButtonValue = currentButtonValue;
  currentButtonValue = digitalRead(BUTTON_PIN);
  digitalWrite(LED_BUILTIN, currentButtonValue);
  if(currentButtonValue != previousButtonValue) 
  {
    if(currentButtonValue == 0) 
    {
      buttonMillis = currentMillis;
      nextDisplay();
    }
    else 
    {
       if((currentMillis - buttonMillis) < DEBOUNCE_DELAY) 
       {
         currentButtonValue = 0;
       }
       if((currentMillis - buttonMillis) > REBOOT_DELAY)
       {
         clearDisplay();
         setCursorPosition(1,1);
         for(int i = 0; i < 32; i++)
         {
           writeToDisplay(BLOCK);
         }
         delay(1000);
         resetFunc();
       }
    }
  }

  //draw display
  if(true) {
    if(currentMillis - lastDisplayMillis >= displayInterval && currentMillis > 500) {
      lastDisplayMillis = currentMillis;
        drawDisplay();
        calculateShiftLight();
    }
  }

  //check for errors
  if(false) {
    if(currentMillis - lastDiagnosticMillis >= diagnosticInterval && currentMillis > 500) {
      lastDiagnosticMillis = currentMillis;
      bool err = calculateErrorLight();
      if(err != inError) {
        inError = err;
        digitalWrite(LED_ERR, err);  
      }
    }
  }
}

char* formatTime(unsigned long milliseconds)
{
  unsigned long seconds = milliseconds / 1000;
  int runHours = seconds / 3600;
  int secsRemaining = seconds % 3600;
  int runMinutes = secsRemaining / 60;
  int runSeconds = secsRemaining % 60;

  char buf[9];
  sprintf(buf,"%02d:%02d:%02d",runHours,runMinutes,runSeconds);
  char* ret = buf;
  return ret;
}

void nextDisplay() 
{
  if(DEBUG)
  {
    Serial.println("next display");
  }
  clearDisplay();
  currentDisplayIndex++;
  if(currentDisplayIndex >= (DISPLAY_COUNT - 1))
  {
    currentDisplayIndex = 0;
    if(engine_clt.currentValue > 159.9)
    {
      currentDisplayIndex = 1;
    }
  }
}

void drawDisplay()
{
  Display* d = allDisplays[currentDisplayIndex];

  if(d->type == single)
  {
    drawSingleGauge(d->gauge1);
  }
  else if(d->type == dual)
  {
    drawDualGauge(d->gauge1, d->gauge2);
  }
  else if(d->type == warmup)
  {
    if(engine_clt.currentValue < 160.0)
    {
      drawWarmup();
    }
    else 
    {
      nextDisplay();
    }
  }
  else if(d->type == runtime)
  {
    drawRuntime();
  }

}

void drawWarmup()
{
  if(startupCLT > 159.9 || engine_clt.currentValue > 159.9)
  {
    nextDisplay();
  }
  else
  {
    writeToDisplay("Warmup", 1, 1);
    writeToDisplay(engine_clt.currentValue, engine_clt.decimalPlaces, 1, 7);
    //drawBar(float lowValue, float highValue, float currentValue, int row, int column, int maxLength)
    drawBar(startupCLT, 160.0, engine_clt.currentValue, 2, 1, 16);
  }
}

void drawRuntime()
{
  writeToDisplay("Runtime", 1, 1);

  char* formattedRuntime = formatTime(millis());
  //Serial.println(formattedRuntime);
  writeToDisplay(formattedRuntime, 1, 9);
}

// void resetCanVariables() {
//   for(int i = 0; i < CAN_MESSAGE_COUNT; i++) 
//   {
//     allCan[i]->filled = false;
//   }
// }

int loadFromCan() {

  // //clearBuffer(&canBuffer[0]);
  // canMsg.cmd      = CMD_RX_DATA;
  // canMsg.pt_data  = &canBuffer[0];
  // canMsg.ctrl.ide = MESSAGE_PROTOCOL; 
  // canMsg.id.std   = 0;
  // canMsg.id.ext   = 0;
  // canMsg.dlc      = MESSAGE_LENGTH;
  // canMsg.ctrl.rtr = MESSAGE_RTR;

  // while(can_cmd(&canMsg) != CAN_CMD_ACCEPTED);

  // // Wait for the command to be accepted by the controller
  // while(can_cmd(&canMsg) != CAN_CMD_ACCEPTED);


  // unsigned long start = millis();


  // bool got1512 = false;
  // bool got1513 = false;
  // bool got1514 = false;
  // bool got1515 = false;
  // bool got1516 = false;

  // int pollCount = 0;
  // while((pollCount < 100) && (!got1512 || !got1513 || !got1514 || !got1515 || !got1516))
  // {
  //   pollCount++;
  //   canMsg.cmd      = CMD_RX_DATA;
  //   canMsg.ctrl.ide = MESSAGE_PROTOCOL; 
  //   canMsg.dlc      = MESSAGE_LENGTH;
  //   canMsg.ctrl.rtr = MESSAGE_RTR;
  //   while(can_cmd(&canMsg) != CAN_CMD_ACCEPTED);
  //   while(can_get_status(&canMsg) == CAN_STATUS_NOT_COMPLETED);
  //   switch(canMsg.id.std)
  //   {
  //     case 1512:
  //       if(!got1512)
  //       {
  //         got1512 = true;
  //         Serial.println("got1512");
  //        // processCanMessage(canMsg);
  //       }
  //       break;
  //     case 1513:
  //       if(!got1513)
  //       {
  //         got1513 = true;
  //         Serial.println("got1513");
  //         //processCanMessage(canMsg);
  //       }
  //       break;
  //     case 1514:
  //       if(!got1514)
  //       {
  //         got1514 = true;
  //         Serial.println("got1514");
  //         //processCanMessage(canMsg);
  //       }
  //       break;
  //     case 1515:
  //       if(!got1515)
  //       {
  //         got1515 = true;
  //         Serial.println("got1515");
  //         //processCanMessage(canMsg);
  //       }
  //       break;
  //     case 1516:
  //       if(!got1516)
  //       {
  //         got1516 = true;
  //         Serial.println("got1516");
  //         //processCanMessage(canMsg);
  //       }
  //       break;
  //     default:
  //       break;
  //   }
    
    
  //   //processCanMessage(canMsg);
  // }
  // Serial.println(pollCount);
  // if(pollCount == 100)
  // {
  //   Serial.println("=====");
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   writeToDisplay("CAN BUS ERROR");
  //   delay(1000);
  //   clearDisplay();
  // }
  // else
  // {
  //   digitalWrite(LED_BUILTIN, LOW);
  //   Serial.println("===============================");
  // }
  // unsigned long end = millis();
  // unsigned long diff = end - start;
  // return 0;
}

void processCanMessages()
{

  engine_map.previousValue = engine_map.currentValue;
  engine_map.currentValue = ((allCanMessages[MSG_1512]->data[0] * 256) + allCanMessages[MSG_1512]->data[1]) / 10.0;
  increment_counter(&engine_map);

  engine_rpm.previousValue = engine_rpm.currentValue;
  //engine_rpm.currentValue = buf[2] * 256 + buf[3];
  //round rpm to nearest 10
  engine_rpm.currentValue = round((allCanMessages[MSG_1512]->data[2] * 256 + allCanMessages[MSG_1512]->data[3]) / 10.0) * 10.0;
  increment_counter(&engine_rpm);
  
  engine_clt.previousValue = engine_clt.currentValue;
  engine_clt.currentValue = (allCanMessages[MSG_1512]->data[4] * 256 + allCanMessages[MSG_1512]->data[5]) / 10.0;
  if(startupCLT == STARTUP_CLT_VALUE)
  {
    startupCLT = engine_clt.currentValue;
  }
  increment_counter(&engine_clt);
  
  engine_tps.previousValue = engine_tps.currentValue;
  engine_tps.currentValue = (allCanMessages[MSG_1512]->data[6] * 256 + allCanMessages[MSG_1512]->data[7]) / 10.0;
  increment_counter(&engine_tps);

  ////////////////////        

  engine_pw1.previousValue = engine_pw1.currentValue;
  engine_pw1.currentValue = (allCanMessages[MSG_1513]->data[0] * 256 + allCanMessages[MSG_1513]->data[1]) / 1000.0;
  increment_counter(&engine_pw1);

  engine_pw2.previousValue = engine_pw2.currentValue;
  engine_pw2.currentValue = (allCanMessages[MSG_1513]->data[2] * 256 + allCanMessages[MSG_1513]->data[3]) / 1000.0;
  increment_counter(&engine_pw2);

  engine_iat.previousValue = engine_iat.currentValue;
  engine_iat.currentValue = (allCanMessages[MSG_1513]->data[4] * 256 + allCanMessages[MSG_1513]->data[5]) / 10.0;
  increment_counter(&engine_iat);

  engine_adv.previousValue = engine_adv.currentValue;
  engine_adv.currentValue = (allCanMessages[MSG_1513]->data[6] * 256 + allCanMessages[MSG_1513]->data[7]) / 10.0;
  increment_counter(&engine_adv);

  ////////////////////
        
  engine_tgt.previousValue = engine_tgt.currentValue;
  engine_tgt.currentValue = (double)allCanMessages[MSG_1514]->data[0] / 10.0;
  increment_counter(&engine_tgt);

  engine_afr.previousValue = engine_afr.currentValue;
  engine_afr.currentValue = (double)allCanMessages[MSG_1514]->data[1] / 10.0;
  increment_counter(&engine_afr);

  engine_ego.previousValue = engine_ego.currentValue;
  engine_ego.currentValue = (allCanMessages[MSG_1514]->data[2] * 256 + allCanMessages[MSG_1514]->data[3]) / 10.0;
  increment_counter(&engine_ego);

  engine_egt.previousValue = engine_egt.currentValue;
  engine_egt.currentValue = (allCanMessages[MSG_1514]->data[4] * 256 + allCanMessages[MSG_1514]->data[5]) / 10.0;
  increment_counter(&engine_egt);

  engine_pws.previousValue = engine_pws.currentValue;
  engine_pws.currentValue = (allCanMessages[MSG_1514]->data[6] * 256 + allCanMessages[MSG_1514]->data[7]) / 1000.0;
  increment_counter(&engine_pws);
        
  ////////////////////

  engine_bat.previousValue = engine_bat.currentValue;
  engine_bat.currentValue = (allCanMessages[MSG_1515]->data[0] * 256 + allCanMessages[MSG_1515]->data[1]) / 10.0;
  increment_counter(&engine_bat);

  //not tested
  engine_sr1.previousValue = engine_sr1.currentValue;
  engine_sr1.currentValue = (allCanMessages[MSG_1515]->data[2] * 256 + allCanMessages[MSG_1515]->data[3]) / 10.0;
  increment_counter(&engine_sr1);

  //not tested
  engine_sr2.previousValue = engine_sr2.currentValue;
  engine_sr2.currentValue = (allCanMessages[MSG_1515]->data[4] * 256 + allCanMessages[MSG_1515]->data[5]) / 10.0;
  increment_counter(&engine_sr2);

  //not tested
  engine_knk.previousValue = engine_knk.currentValue;
  engine_knk.currentValue = (allCanMessages[MSG_1515]->data[6] * 256) / 10.0;
  increment_counter(&engine_knk);

  ////////////////////

  //not tested
  engine_vss.previousValue = engine_vss.currentValue;
  engine_vss.currentValue = (allCanMessages[MSG_1516]->data[0] * 256 + allCanMessages[MSG_1516]->data[1]) / 10.0;
  increment_counter(&engine_vss);

  //not tested
  engine_tcr.previousValue = engine_tcr.currentValue;
  engine_tcr.currentValue = (allCanMessages[MSG_1516]->data[2] * 256 + allCanMessages[MSG_1516]->data[3]) / 10.0;
  increment_counter(&engine_tcr);

  engine_lct.previousValue = engine_lct.currentValue;
  engine_lct.previousValue = (allCanMessages[MSG_1516]->data[4] * 256 + allCanMessages[MSG_1516]->data[5]) / 10.0;
  increment_counter(&engine_lct);
        
}

void serialPrintCanData(volatile canData *canMsg)
{
  char textBuffer[50] = {0};
  //sprintf(textBuffer,"id %04x ",canMsg->id);
  //Serial.print(textBuffer);
  sprintf(textBuffer,"data ");
  Serial.print(textBuffer);
  for (int i =0; i < 8; i++){
    //sprintf(textBuffer,"%02X ", canMsg->data[i]);
    //Serial.print(textBuffer);
  }
  Serial.print("\r\n");
}

void serialPrintData(volatile st_cmd_t *msg){
  char textBuffer[50] = {0};
  if (msg->ctrl.ide>0){
    sprintf(textBuffer,"id %d ",msg->id.ext);
  }
  else
  {
    sprintf(textBuffer,"id %04x ",msg->id.std);
  }
  Serial.print(textBuffer);
  
  //  IDE
  sprintf(textBuffer,"ide %d ",msg->ctrl.ide);
  Serial.print(textBuffer);
  //  RTR
  sprintf(textBuffer,"rtr %d ",msg->ctrl.rtr);
  Serial.print(textBuffer);
  //  DLC
  sprintf(textBuffer,"dlc %d ",msg->dlc);
  Serial.print(textBuffer);
  //  Data
  sprintf(textBuffer,"data ");
  Serial.print(textBuffer);
  
  for (int i =0; i<msg->dlc; i++){
    sprintf(textBuffer,"%02X ",msg->pt_data[i]);
    Serial.print(textBuffer);
  }
  Serial.print("\r\n");
}

void increment_counter(EngineVariable* engine) {
  if(engine->currentValue > engine->maximum) {
    engine->highCount++;
  }
  else if(engine->currentValue < engine->minimum) {
    engine->lowCount++;
  }
  else {
    engine->goodCount++;
  }  
}

void drawBar(EngineVariable* engineVar, int row, int column, int maxLength) {
  
  int length = map(engineVar->currentValue, engineVar->minimum, engineVar->maximum, 0, maxLength);

  for(int i = 0; i < maxLength; i++) {
    if(i > length) {
      writeSpecialToDisplay(SPACE, row, column+i);
    }
    else {
      writeSpecialToDisplay(BLOCK, row, column+i);
    }
  }
}

void drawBar(float lowValue, float highValue, float currentValue, int row, int column, int maxLength)
{
  int length = map(currentValue, lowValue, highValue, 0, maxLength);
  for(int i = 0; i < maxLength; i++) {
    if(i > length) {
      writeSpecialToDisplay(SPACE, row, column+i);
    }
    else {
      writeSpecialToDisplay(BLOCK, row, column+i);
    }
  }
}

bool calculateErrorLight() {
  byte len = 20; //sizeof(allGauges);
  unsigned long badCount;
  unsigned long totalCount;
  byte percent;
  bool inError = false;
  
  for(byte i = 0; i < len; i++) {
    badCount = allGauges[i]->lowCount + allGauges[i]->highCount;
    totalCount = badCount + allGauges[i]->goodCount;
    percent = badCount * 100 / totalCount;
    if(percent > 4) {
      inError = true;
    }
    if(totalCount > 1000) {
      allGauges[i]->lowCount = allGauges[i]->lowCount * 0.8;
      allGauges[i]->highCount = allGauges[i]->highCount * 0.8;
      allGauges[i]->goodCount = allGauges[i]->goodCount * 0.8;
    }
  }
  return inError;
}

void calculateShiftLight()
{
  previousShiftLight = shiftLight;
  int shiftLightValue = 0;
  if(engine_tps.currentValue > 75)
  {
    shiftLightValue = SHIFT_LIGHT_FROM_REDLINE_WOT;
  }
  else 
  {
    shiftLightValue = SHIFT_LIGHT_FROM_REDLINE_CRUISE;
  }
  shiftLight = (engine_rpm.currentValue + shiftLightValue) > engine_rpm.maximum;

  if(shiftLight == true && previousShiftLight == false)
  {
    digitalWrite(LED_SHIFT, HIGH);
    if(DEBUG)
    {
      Serial.println("Shift Light ON");
    }
  }
  else if(shiftLight == false && previousShiftLight == true)
  {
    digitalWrite(LED_SHIFT, LOW);
    if(DEBUG)
    {
      Serial.println("Shift Light OFF");
    }
  }

}

void drawSingleGauge(EngineVariable* gauge)
{
  char* gaugeLabel = gauge->shortLabel;
  float gaugeValue = gauge->currentValue;
  int gaugeDecimal = gauge->decimalPlaces;
  writeToDisplay(gaugeLabel, 1, 1);
  writeToDisplay(gaugeValue, gaugeDecimal, 1, 5);
  drawBar(gauge, 2, 1, 16);
}

void drawDualGauge(EngineVariable* gauge1, EngineVariable* gauge2)
{
  char* gauge1Label = gauge1->shortLabel;
  float gauge1Value = gauge1->currentValue;
  int gauge1Decimal = gauge1->decimalPlaces;
  writeToDisplay(gauge1Label, 1, 1);
  writeToDisplay(gauge1Value, gauge1Decimal, 1, 5);
  drawBar(gauge1, 1, 9, 8);

  char* gauge2Label = gauge2->shortLabel;
  float gauge2Value = gauge2->currentValue;
  int gauge2Decimal = gauge2->decimalPlaces;
  writeToDisplay(gauge2Label, 2, 1);
  writeToDisplay(gauge2Value, gauge2Decimal, 2, 5);
  drawBar(gauge2, 2, 9, 8);
}

void bootAnimation() {
  int smallDelay = 30;
  int bigDelay = 200;

  writeSpecialToDisplay(BLOCK, 1, 1);
  writeSpecialToDisplay(BLOCK, 2, 16);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 2);
  writeSpecialToDisplay(BLOCK, 2, 15);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 3);
  writeSpecialToDisplay(BLOCK, 2, 14);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 4);
  writeSpecialToDisplay(BLOCK, 2, 13);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 5);
  writeSpecialToDisplay(BLOCK, 2, 12);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 6);
  writeSpecialToDisplay(BLOCK, 2, 11);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 7);
  writeSpecialToDisplay(BLOCK, 2, 10);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 8);
  writeSpecialToDisplay(BLOCK, 2, 9);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 9);
  writeSpecialToDisplay(BLOCK, 2, 8);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 10);
  writeSpecialToDisplay(BLOCK, 2, 7);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 11);
  writeSpecialToDisplay(BLOCK, 2, 6);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 12);
  writeSpecialToDisplay(BLOCK, 2, 5);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 13);
  writeSpecialToDisplay(BLOCK, 2, 4);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 14);
  writeSpecialToDisplay(BLOCK, 2, 3);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 15);
  writeSpecialToDisplay(BLOCK, 2, 2);
  delay(smallDelay);
  writeSpecialToDisplay(BLOCK, 1, 16);
  writeSpecialToDisplay(BLOCK, 2, 1);

  delay(bigDelay);

  writeSpecialToDisplay(SPACE, 1, 1);
  writeSpecialToDisplay(SPACE, 2, 16);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 2);
  writeSpecialToDisplay(SPACE, 2, 15);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 3);
  writeSpecialToDisplay(SPACE, 2, 14);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 4);
  writeSpecialToDisplay(SPACE, 2, 13);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 5);
  writeSpecialToDisplay(SPACE, 2, 12);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 6);
  writeSpecialToDisplay(SPACE, 2, 11);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 7);
  writeSpecialToDisplay(SPACE, 2, 10);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 8);
  writeSpecialToDisplay(SPACE, 2, 9);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 9);
  writeSpecialToDisplay(SPACE, 2, 8);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 10);
  writeSpecialToDisplay(SPACE, 2, 7);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 11);
  writeSpecialToDisplay(SPACE, 2, 6);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 12);
  writeSpecialToDisplay(SPACE, 2, 5);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 13);
  writeSpecialToDisplay(SPACE, 2, 4);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 14);
  writeSpecialToDisplay(SPACE, 2, 3);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 15);
  writeSpecialToDisplay(SPACE, 2, 2);
  delay(smallDelay);
  writeSpecialToDisplay(SPACE, 1, 16);
  writeSpecialToDisplay(SPACE, 2, 1);

}