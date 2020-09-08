#include <Arduino.h>
#include "Can485DisplayHelper.h"
#include <ASTCanLib.h>
#include <math.h>
#include "AutoCAN.h"

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

struct CanVariable
{
  int id;
  bool filled;
  byte data[8];
};

int canIDs[] = {1512,1513,1514,1515,1516};

const byte CAN_MESSAGE_COUNT = 5;
CanVariable* allCan[CAN_MESSAGE_COUNT];
CanVariable can1512 = {1512, false, NULL};
CanVariable can1513 = {1513, false, NULL};
CanVariable can1514 = {1514, false, NULL};
CanVariable can1515 = {1515, false, NULL};
CanVariable can1516 = {1516, false, NULL};

st_cmd_t canMsg;
uint8_t canBuffer[8] = {};

#define MESSAGE_PROTOCOL  0     // CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
#define MESSAGE_LENGTH    8     // Data length: 8 bytes
#define MESSAGE_RTR       0     // rtr bit


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
EngineVariable engine_vss   = {"VSS", 0.0, 0.0, 0.0, 160.0, 0, 0, 0, 0};      //vehicle speed
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

void setup() {

  DisplayInit();  

  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_SHIFT, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  allCan[0] = &can1512;
  allCan[1] = &can1513;
  allCan[2] = &can1514;
  allCan[3] = &can1515;
  allCan[4] = &can1516;

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

  if(DEBUG) {
    Serial.println("CAN bus");
  }
  clearDisplay();
  bootAnimation();
}

void loop() {

  loadFromCan();

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
  if(true) {
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
  Serial.println(formattedRuntime);
  writeToDisplay(formattedRuntime, 1, 9);
}

void resetCanVariables() {
  for(int i = 0; i < CAN_MESSAGE_COUNT; i++) 
  {
    allCan[i]->filled = false;
  }
}

int loadFromCan() {

  clearBuffer(&canBuffer[0]);
  canMsg.cmd      = CMD_RX_DATA;
  canMsg.pt_data  = &canBuffer[0];
  canMsg.ctrl.ide = MESSAGE_PROTOCOL; 
  canMsg.id.std   = 0;
  canMsg.id.ext   = 0;
  canMsg.dlc      = MESSAGE_LENGTH;
  canMsg.ctrl.rtr = MESSAGE_RTR;

  // Wait for the command to be accepted by the controller
  //while(can_cmd(&canMsg) != CAN_CMD_ACCEPTED);


  unsigned long start = millis();


  bool got1512 = false;
  bool got1513 = false;
  bool got1514 = false;
  bool got1515 = false;
  bool got1516 = false;

  int pollCount = 0;
  while((pollCount < 100) && (!got1512 || !got1513 || !got1514 || !got1515 || !got1516))
  {
    pollCount++;
    canMsg.cmd      = CMD_RX_DATA;
    canMsg.ctrl.ide = MESSAGE_PROTOCOL; 
    canMsg.dlc      = MESSAGE_LENGTH;
    canMsg.ctrl.rtr = MESSAGE_RTR;
    while(can_cmd(&canMsg) != CAN_CMD_ACCEPTED);
    while(can_get_status(&canMsg) == CAN_STATUS_NOT_COMPLETED);
    switch(canMsg.id.std)
    {
      case 1512:
        if(!got1512)
        {
          got1512 = true;
          Serial.println("got1512");
          processCanMessage(canMsg);
        }
        break;
      case 1513:
        if(!got1513)
        {
          got1513 = true;
          Serial.println("got1513");
          processCanMessage(canMsg);
        }
        break;
      case 1514:
        if(!got1514)
        {
          got1514 = true;
          Serial.println("got1514");
          processCanMessage(canMsg);
        }
        break;
      case 1515:
        if(!got1515)
        {
          got1515 = true;
          Serial.println("got1515");
          processCanMessage(canMsg);
        }
        break;
      case 1516:
        if(!got1516)
        {
          got1516 = true;
          Serial.println("got1516");
          processCanMessage(canMsg);
        }
        break;
      default:
        break;
    }
    
    
    //processCanMessage(canMsg);
  }
  Serial.println(pollCount);
  if(pollCount == 100)
  {
    Serial.println("=====");
    digitalWrite(LED_BUILTIN, HIGH);
    writeToDisplay("CAN BUS ERROR");
    delay(1000);
    clearDisplay();
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("===============================");
  }
  unsigned long end = millis();
  unsigned long diff = end - start;
  return 0;
}

void processCanMessage(st_cmd_t msg)
{
      switch(msg.id.std) {
      case 1512:
        engine_map.previousValue = engine_map.currentValue;
        engine_map.currentValue = ((msg.pt_data[0] * 256) + msg.pt_data[1]) / 10.0;
        increment_counter(&engine_map);

        engine_rpm.previousValue = engine_rpm.currentValue;
        //engine_rpm.currentValue = buf[2] * 256 + buf[3];
        //round rpm to nearest 10
        engine_rpm.currentValue = round((msg.pt_data[2] * 256 + msg.pt_data[3]) / 10.0) * 10.0;
        increment_counter(&engine_rpm);
        
        engine_clt.previousValue = engine_clt.currentValue;
        engine_clt.currentValue = (msg.pt_data[4] * 256 + msg.pt_data[5]) / 10.0;
        if(startupCLT == STARTUP_CLT_VALUE)
        {
          startupCLT = engine_clt.currentValue;
        }
        increment_counter(&engine_clt);
        
        engine_tps.previousValue = engine_tps.currentValue;
        engine_tps.currentValue = (msg.pt_data[6] * 256 + msg.pt_data[7]) / 10.0;
        increment_counter(&engine_tps);
        
        break;
      case 1513:
        engine_pw1.previousValue = engine_pw1.currentValue;
        engine_pw1.currentValue = (msg.pt_data[0] * 256 + msg.pt_data[1]) / 1000.0;
        increment_counter(&engine_pw1);

        engine_pw2.previousValue = engine_pw2.currentValue;
        engine_pw2.currentValue = (msg.pt_data[2] * 256 + msg.pt_data[3]) / 1000.0;
        increment_counter(&engine_pw2);

        engine_iat.previousValue = engine_iat.currentValue;
        engine_iat.currentValue = (msg.pt_data[4] * 256 + msg.pt_data[5]) / 10.0;
        increment_counter(&engine_iat);

        engine_adv.previousValue = engine_adv.currentValue;
        engine_adv.currentValue = (msg.pt_data[6] * 256 + msg.pt_data[7]) / 10.0;
        increment_counter(&engine_adv);
        
        break;
      case 1514:
        engine_tgt.previousValue = engine_tgt.currentValue;
        engine_tgt.currentValue = (double)msg.pt_data[0] / 10.0;
        increment_counter(&engine_tgt);

        engine_afr.previousValue = engine_afr.currentValue;
        engine_afr.currentValue = (double)msg.pt_data[1] / 10.0;
        increment_counter(&engine_afr);

        engine_ego.previousValue = engine_ego.currentValue;
        engine_ego.currentValue = (msg.pt_data[2] * 256 + msg.pt_data[3]) / 10.0;
        increment_counter(&engine_ego);

        engine_egt.previousValue = engine_egt.currentValue;
        engine_egt.currentValue = (msg.pt_data[4] * 256 + msg.pt_data[5]) / 10.0;
        increment_counter(&engine_egt);

        engine_pws.previousValue = engine_pws.currentValue;
        engine_pws.currentValue = (msg.pt_data[6] * 256 + msg.pt_data[7]) / 1000.0;
        increment_counter(&engine_pws);
        
        break;
      case 1515:
        engine_bat.previousValue = engine_bat.currentValue;
        engine_bat.currentValue = (msg.pt_data[0] * 256 + msg.pt_data[1]) / 10.0;
        increment_counter(&engine_bat);

        //not tested
        engine_sr1.previousValue = engine_sr1.currentValue;
        engine_sr1.currentValue = (msg.pt_data[2] * 256 + msg.pt_data[3]) / 10.0;
        increment_counter(&engine_sr1);

        //not tested
        engine_sr2.previousValue = engine_sr2.currentValue;
        engine_sr2.currentValue = (msg.pt_data[4] * 256 + msg.pt_data[5]) / 10.0;
        increment_counter(&engine_sr2);

        //not tested
        engine_knk.previousValue = engine_knk.currentValue;
        engine_knk.currentValue = (msg.pt_data[6] * 256) / 10.0;
        increment_counter(&engine_knk);

        break;
      case 1516:
        //not tested
        engine_vss.previousValue = engine_vss.currentValue;
        engine_vss.currentValue = (msg.pt_data[0] * 256 + msg.pt_data[1]) / 10.0;
        increment_counter(&engine_vss);

        //not tested
        engine_tcr.previousValue = engine_tcr.currentValue;
        engine_tcr.currentValue = (msg.pt_data[2] * 256 + msg.pt_data[3]) / 10.0;
        increment_counter(&engine_tcr);

        engine_lct.previousValue = engine_lct.currentValue;
        engine_lct.previousValue = (msg.pt_data[4] * 256 + msg.pt_data[5]) / 10.0;
        increment_counter(&engine_lct);
        
        break;
      default:
        //do nothing
        break;
    }

}

void serialPrintData(st_cmd_t *msg){
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
  Serial.print(gauge1Label);
  Serial.print("~");
  Serial.print(gauge1Value);
  Serial.print("~");
  Serial.println(gauge1Decimal);
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