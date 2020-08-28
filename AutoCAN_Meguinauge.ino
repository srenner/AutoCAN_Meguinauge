#include <Arduino.h>
#include "Can485DisplayHelper.h"
#include <ASTCanLib.h>

#define BLOCK 255 //block character to build bar graphs

// SET UP PINS ///////////////////////////////////////////

//This version of AltSoftSerial hard-codes the pins to 9 (rx) and 5(tx)
//This also disables PWM on 6 and 7

const byte LED_ERR = LED_BUILTIN;
const byte LED_SHIFT = 15;
const byte SPI_CS_PIN = 10;
const byte GAUGE_PIN = 18;


// BUILD ENGINE VARIABLES ///////////////////////////////////////

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
EngineVariable engine_rpm   = {"RPM", 0.0, 0.0, 0.0, 6500.0, 0, 0, 0, 0};     //engine rpm
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
EngineVariable engine_bat   = {"BAT", 0.0, 0.0, 8.0, 18.0, 1, 0, 0, 0};       //battery voltage
EngineVariable engine_sr1   = {"SR1", 0.0, 0.0, 0.0, 999.0, 1, 0, 0, 0};      //generic sensor 1
EngineVariable engine_sr2   = {"SR2", 0.0, 0.0, 0.0, 999.0, 1, 0, 0, 0};      //generic sensor 2
EngineVariable engine_knk   = {"KNK", 0.0, 0.0, 0.0, 50.0, 1, 0, 0, 0};       //knock ignition retard
EngineVariable engine_vss   = {"VSS", 0.0, 0.0, 0.0, 160.0, 0, 0, 0, 0};      //vehicle speed
EngineVariable engine_tcr   = {"TCR", 0.0, 0.0, 0.0, 50.0, 1, 0, 0, 0};       //traction control ignition retard
EngineVariable engine_lct   = {"LCT", 0.0, 0.0, 0.0, 50.0, 1, 0, 0, 0};       //launch control timing

// LOOP TIMER VARIABLES ///////////////////////////////

unsigned long currentMillis = 0;
byte displayInterval = 100;
unsigned long lastDisplayMillis = 0;
unsigned int diagnosticInterval = 5000;
unsigned long lastDiagnosticMillis = 0;

// GAUGE ARRAYS FOR EACH MODE ////////////////////////

const byte DUAL_LEN = 3;
const byte QUAD_LEN = 2;
const byte OCTO_LEN = 1;

byte dualIndex = 0;
byte quadIndex = 0;
byte octoIndex = 0;

EngineVariable* dualModeGauges[DUAL_LEN][2];
EngineVariable* quadModeGauges[QUAD_LEN][4];
EngineVariable* octoModeGauges[OCTO_LEN][8];

// MODE VARIABLES ////////////////////////////////////

enum Mode {
  single,
  dual,
  sensor_warmup,
  engine_warmup,
  diag
};
Mode currentMode;

bool dualModeReady = false;
bool quadModeReady = false;
bool octoModeReady = false;
bool diagModeReady = false;

bool inError = false;

bool currentModeButton = 1;
bool previousModeButton = 1;
unsigned long modeButtonMillis = 0;

bool currentGaugeButton = 1;
bool previousGaugeButton = 1;
unsigned long gaugeButtonMillis = 0;

const byte DEBOUNCE_DELAY = 250;

MCP_CAN CAN(SPI_CS_PIN); 

void setup() {
  

  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_SHIFT, OUTPUT);
  pinMode(GAUGE_PIN, INPUT_PULLUP);
  //digitalWrite(MODE_PIN, INPUT_PULLUP);

  dualModeGauges[0][0] = &engine_rpm;
  dualModeGauges[0][1] = &engine_afr;

  dualModeGauges[1][0] = &engine_map;
  dualModeGauges[1][1] = &engine_afr;

  dualModeGauges[2][0] = &engine_afr;
  dualModeGauges[2][1] = &engine_tgt;
  
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
  


  Serial.begin(115200);
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    //lcd.print(F("Waiting for CAN bus"));
    delay(100);
  }
    //lcd.print(F("Connected to ECU"));
}

void loop() {
  load_from_can();
  
  currentMillis = millis();

  //draw display
  if(currentMillis - lastDisplayMillis >= displayInterval && currentMillis > 500) {
    lastDisplayMillis = currentMillis;

      //do the needful

  }

  //check for errors
  if(currentMillis - lastDiagnosticMillis >= diagnosticInterval && currentMillis > 500) {
    lastDiagnosticMillis = currentMillis;
    bool err = calculate_error_light();
    if(err != inError) {
      inError = err;
      digitalWrite(LED_ERR, err);  
    }
  }

  previousModeButton = currentModeButton;
  currentModeButton = digitalRead(GAUGE_PIN);


  previousGaugeButton = currentGaugeButton;
  currentGaugeButton = digitalRead(GAUGE_PIN);
  if(currentGaugeButton != previousGaugeButton) {
    if(currentGaugeButton == 0) {
      gaugeButtonMillis = currentMillis;
      Serial.println("next gauge");
      next_gauge();
    }
    else {
      if((currentMillis - gaugeButtonMillis) < DEBOUNCE_DELAY) {
        currentGaugeButton = 0;
      }
    }
  }
}



void next_gauge() {

  clear_mode();
  
  if(currentMode == dual) {
    if(dualIndex == (DUAL_LEN - 1)) {
      dualIndex = 0; 
    }
    else {
      dualIndex++;
    }
  }

  else {
    Serial.println("unknown mode");
  }
}

void clear_mode() {

}

void draw_dual_gauges() {
  //dual
}



void load_from_can() {
  unsigned char len = 0;
  unsigned char buf[8];
  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    unsigned int canId = CAN.getCanId();
    
    switch(canId) {
      case 1512:
        engine_map.previousValue = engine_map.currentValue;
        engine_map.currentValue = ((buf[0] * 256) + buf[1]) / 10.0;
        increment_counter(&engine_map);

        engine_rpm.previousValue = engine_rpm.currentValue;
        //engine_rpm.currentValue = buf[2] * 256 + buf[3];
        //round rpm to nearest 10
        engine_rpm.currentValue = round((buf[2] * 256 + buf[3]) / 10.0) * 10.0;
        increment_counter(&engine_rpm);
        
        engine_clt.previousValue = engine_clt.currentValue;
        engine_clt.currentValue = (buf[4] * 256 + buf[5]) / 10.0;
        increment_counter(&engine_clt);
        
        engine_tps.previousValue = engine_tps.currentValue;
        engine_tps.currentValue = (buf[6] * 256 + buf[7]) / 10.0;
        increment_counter(&engine_tps);
        
        break;
      case 1513:
        engine_pw1.previousValue = engine_pw1.currentValue;
        engine_pw1.currentValue = (buf[0] * 256 + buf[1]) / 1000.0;
        increment_counter(&engine_pw1);

        engine_pw2.previousValue = engine_pw2.currentValue;
        engine_pw2.currentValue = (buf[2] * 256 + buf[3]) / 1000.0;
        increment_counter(&engine_pw2);

        engine_iat.previousValue = engine_iat.currentValue;
        engine_iat.currentValue = (buf[4] * 256 + buf[5]) / 10.0;
        increment_counter(&engine_iat);

        engine_adv.previousValue = engine_adv.currentValue;
        engine_adv.currentValue = (buf[6] * 256 + buf[7]) / 10.0;
        increment_counter(&engine_adv);
        
        break;
      case 1514:
        engine_tgt.previousValue = engine_tgt.currentValue;
        engine_tgt.currentValue = (double)buf[0] / 10.0;
        increment_counter(&engine_tgt);

        engine_afr.previousValue = engine_afr.currentValue;
        engine_afr.currentValue = (double)buf[1] / 10.0;
        increment_counter(&engine_afr);

        engine_ego.previousValue = engine_ego.currentValue;
        engine_ego.currentValue = (buf[2] * 256 + buf[3]) / 10.0;
        increment_counter(&engine_ego);

        engine_egt.previousValue = engine_egt.currentValue;
        engine_egt.currentValue = (buf[4] * 256 + buf[5]) / 10.0;
        increment_counter(&engine_egt);

        engine_pws.previousValue = engine_pws.currentValue;
        engine_pws.currentValue = (buf[6] * 256 + buf[7]) / 1000.0;
        increment_counter(&engine_pws);
        
        break;
      case 1515:
        engine_bat.previousValue = engine_bat.currentValue;
        engine_bat.currentValue = (buf[0] * 256 + buf[1]) / 10.0;
        increment_counter(&engine_bat);

        //not tested
        engine_sr1.previousValue = engine_sr1.currentValue;
        engine_sr1.currentValue = (buf[2] * 256 + buf[3]) / 10.0;
        increment_counter(&engine_sr1);

        //not tested
        engine_sr2.previousValue = engine_sr2.currentValue;
        engine_sr2.currentValue = (buf[4] * 256 + buf[5]) / 10.0;
        increment_counter(&engine_sr2);

        //not tested
        engine_knk.previousValue = engine_knk.currentValue;
        engine_knk.currentValue = (buf[6] * 256) / 10.0;
        increment_counter(&engine_knk);

        break;
      case 1516:
        //not tested
        engine_vss.previousValue = engine_vss.currentValue;
        engine_vss.currentValue = (buf[0] * 256 + buf[1]) / 10.0;
        increment_counter(&engine_vss);

        //not tested
        engine_tcr.previousValue = engine_tcr.currentValue;
        engine_tcr.currentValue = (buf[2] * 256 + buf[3]) / 10.0;
        increment_counter(&engine_tcr);

        engine_lct.previousValue = engine_lct.currentValue;
        engine_lct.previousValue = (buf[4] * 256 + buf[5]) / 10.0;
        increment_counter(&engine_lct);
        
        break;
      default:
        //do nothing
        break;
    }
  }
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

//note: using log10 would work but this is faster
bool is_current_value_shorter(EngineVariable engine) {
  int roundedCurrent = round(engine.currentValue);
  int roundedPrevious = round(engine.previousValue);
  byte currentLength;
  if(roundedCurrent >= 10000) {
    currentLength = 5;
  }
  else if(roundedCurrent >= 1000) {
    currentLength = 4;
  }
  else if(roundedCurrent >= 100) {
    currentLength = 3;
  }
  else if(roundedCurrent >= 10) {
    currentLength = 2;
  }
  else {
    currentLength = 1;
  }
  if(engine.decimalPlaces > 0) {
    currentLength++;
    currentLength+= engine.decimalPlaces;
  }

  byte previousLength;
  if(roundedPrevious >= 10000) {
    previousLength = 5;
  }
  else if(roundedPrevious >= 1000) {
    previousLength = 4;
  }
  else if(roundedPrevious >= 100) {
    previousLength = 3;
  }
  else if(roundedPrevious >= 10) {
    previousLength = 2;
  }
  else {
    previousLength = 1;
  }
  if(engine.decimalPlaces > 0) {
    previousLength++;
    previousLength += engine.decimalPlaces;
  }

  return currentLength < previousLength;
}

void draw_bar(EngineVariable engineVar, byte row, byte column) {
  //todo
}

bool calculate_error_light() {
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




