#include <Arduino.h>
#include <ASTCanLib.h>
#include <math.h>
#include <AutoCAN.h>
#include <LiquidCrystal.h>

/*
  CONFIGURATION NOTES
  ===================
  - setting DEBUG true outputs messages to the serial port
*/

#define DEBUG false           //print out debug messages on serial port
#define DEBUG_CAN false       //print can message counts  to serial
#define DEBUG_ENG false       //print engine variables to serial

#define USE_SENSORHUB_VSS true      //true to read vss from AutoCAN_SensorHub instead of MegaSquirt
#define USE_SENSORHUB_FPR false     //true to read fuel pressure from AutoCAN_SensorHub
#define USE_SENSORHUB_OIL false     //true to read oil pressure from AutoCAN_SensorHub
#define USE_SENSORHUB_GPS true      //true to read gps coordinates and time/date from AutoCAN_SensorHub
#define USE_SENSORHUB_ACCEL false   //true to read accelerometer data from AutoCAN_SensorHub
#define USE_SENSORHUB_COMPASS true

#define USE_24HOUR_TIME true        //13:00 vs 01:00pm, etc.
#define USE_NOISY_BUTTON true       //short beep on button press

// CAN BUS OBJECTS /////////////////////////////////////////////////////////////

uint8_t canBuffer[8] = {};

#define MESSAGE_PROTOCOL  0     // CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
#define MESSAGE_LENGTH    8     // Data length: 8 bytes
#define MESSAGE_RTR       0     // rtr bit

volatile uint32_t canCount = 0;
volatile uint32_t canUnhandledCount = 0;
volatile float sensorHubMph = 0.0;

volatile st_cmd_t canMsg;

typedef struct {
  int16_t id;
  uint32_t counter;
  uint8_t* data;
} canData;

volatile canData* allCanMessages[5];  //array of all MegaSquirt CAN messages we are interested in receiving

volatile canData canBase;
volatile canData canPlus1;
volatile canData canPlus2;
volatile canData canPlus3;
volatile canData canPlus4;

uint8_t canBufferBase[8] = {};
uint8_t canBufferPlus1[8] = {};
uint8_t canBufferPlus2[8] = {};
uint8_t canBufferPlus3[8] = {};
uint8_t canBufferPlus4[8] = {};

volatile canData canTemp;
uint8_t canBufferTemp[8] = {};

// SET UP PINS /////////////////////////////////////////////////////////////////

const uint8_t BUZZER_PIN = 4;   // sound making thing
const uint8_t LED_ERR = 7;      // 'check engine' light
const uint8_t LED_SHIFT = 8;    // shift light
const uint8_t BUTTON_PIN = 9;   // pushbutton to cycle through modes

const uint8_t RS_PIN = 14;
const uint8_t EN_PIN = 15;
const uint8_t DB7_PIN = 13;
const uint8_t DB6_PIN = 12;
const uint8_t DB5_PIN = 10;
const uint8_t DB4_PIN = 11;

LiquidCrystal lcd(RS_PIN, EN_PIN, DB4_PIN, DB5_PIN, DB6_PIN, DB7_PIN);

// CUSTOM LCD CHARACTERS ///////////////////////////////////////////////////////

uint8_t fill1[8] = {
  0x10,
  0x10,
  0x10,
  0x10,
  0x10,
  0x10,
  0x10,
  0x10
};
uint8_t fill2[8] = {
  0x18,
  0x18,
  0x18,
  0x18,
  0x18,
  0x18,
  0x18,
  0x18
};
uint8_t fill3[8] = {
  0x1C,
  0x1C,
  0x1C,
  0x1C,
  0x1C,
  0x1C,
  0x1C,
  0x1C
};
uint8_t fill4[8] = {
  0x1E,
  0x1E,
  0x1E,
  0x1E,
  0x1E,
  0x1E,
  0x1E,
  0x1E
};
uint8_t fill5[8] = {
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F
};
uint8_t fillMiddle[8] = {
  0x04,
  0x04,
  0x04,
  0x04,
  0x04,
  0x04,
  0x04,
  0x04  
};
uint8_t fillNothing[8] = {
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00
};

// BUILD ENGINE VARIABLES //////////////////////////////////////////////////////

bool shiftLight = false;
bool previousShiftLight = false;

bool currentButtonValue = 1;
bool previousButtonValue = 1;
uint32_t buttonMillis = 0;

// WARMUP VARIABLES ////////////////////////////////////////////////////////////

const float STARTUP_CLT_VALUE = -200.0;
float startupCLT = STARTUP_CLT_VALUE;

// LOOP TIMER VARIABLES ////////////////////////////////////////////////////////

uint32_t currentMillis = 0;
uint32_t previousMillis = 0;
uint32_t startPolling = 0;
uint32_t endPolling = 0;
uint16_t pollCount = 0;
uint16_t maxPollingDelay = 0;
uint16_t pollingDelayLimit = 10;
uint8_t displayInterval = 100;
uint32_t lastDisplayMillis = 0;
uint16_t diagnosticInterval = 5000;
uint32_t lastDiagnosticMillis = 0;

uint8_t clickBuzzerInterval = 1;

uint8_t shortBuzzerInterval = 25;
uint32_t shortBuzzerStartMillis = 0;

// MODE VARIABLES //////////////////////////////////////////////////////////////

enum DisplayType {
  warmup,
  runtime,
  single,
  dual,
  zerotosixty,
  sixtytozero,
  quartermile,
  gforce,
  diagnostic
};

struct Display
{
  DisplayType type;
  EngineVariable* gauge1;
  EngineVariable* gauge2;
};

const uint8_t DISPLAY_COUNT = 19;
uint8_t currentDisplayIndex = 0;
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
Display display_060       = {zerotosixty, NULL, NULL};
Display display_14m       = {quartermile, NULL, NULL};
Display display_600       = {sixtytozero, NULL, NULL};

bool inError = false;

const uint8_t DEBOUNCE_DELAY = 250;
const int REBOOT_DELAY = 2000;
const int SHIFT_LIGHT_FROM_REDLINE_WOT = 500;
const int SHIFT_LIGHT_FROM_REDLINE_CRUISE = 1000;

// SENSORHUB VARIABLES /////////////////////////////////////////////////////////

volatile datetime datetimeCAN;
datetime datetimeSafe;

uint8_t compassDirectionIndexCAN = 8;
uint8_t compassDirectionIndexSafe = 8;

uint32_t vssPulseCAN = 0;
uint32_t vssPulseSafe = 0;

void(* resetFunc) (void) = 0; //declare killswitch function

void clearBuf(volatile uint8_t* Buffer){
  for (int i=0; i<8; i++){
    Buffer[i] = 0x00;
  }
}

// perf variables //////////////////////////////////////////////////////////////

// bool perfActive = false;
// uint32_t perfStartMillis = 0;
// uint32_t perfEndMillis = 0;
// uint32_t ztsElapsedMillis = 0;

// uint32_t perfStartPulses = 0;
// uint32_t perfEndPulses = 0;
// uint16_t perfElapsedPulses = 0;
// uint16_t perfQuarterMilePulses = VSS_PULSE_PER_MILE / 4;

typedef struct
{
  bool isActive;
  uint32_t startMillis;
  uint32_t endMillis;
  uint32_t elapsedMillis;
  float mph;
  uint32_t startPulses;
  uint32_t endPulses;
  uint16_t elapsedPulses;
} PerformanceTracker;

PerformanceTracker tracker060; //0-60 time
PerformanceTracker tracker600; //60-0 time
PerformanceTracker tracker14;  //quarter mile time and speed

// void resetPerformanceTracker(PerformanceTracker* t)
// {
//   t->isActive = false;
//   t->startMillis = 0;
//   t->endMillis = 0;
//   t->elapsedMillis = 0;
//   t->mph = 0.0;
//   t->startPulses = 0;
//   t->endPulses = 0;
//   t->elapsedPulses = 0;
// }

ISR(CANIT_vect) {
  canCount++;

  unsigned i;   
  char save_canpage=CANPAGE;   
  
  unsigned mob = CANHPMOB; // get highest prio mob   
  CANPAGE = mob & 0xf0;   
  mob >>= 4; // -> mob number 0..15   
  //ASSERT( (CANSTMOB & ~0xa0) ==0); // allow only RX ready and DLC warning   
    
  canTemp.id = (CANIDT2>>5) | (CANIDT1 <<3);
  
  register char length; 
  length = CANCDMOB & 0x0f;
  for (i = 0; i <length; ++i)   
  {
    canTemp.data[i] = CANMSG;
  }   
  
  CANSTMOB = 0;           // reset INT reason   
  CANCDMOB = 0x80;        // re-enable RX on this channel   
  CANPAGE = save_canpage; // restore CANPAGE   

  if(true) 
  {
    switch(canTemp.id)
    {
      case MS_BASE_ID:
        allCanMessages[MSG_MS_BASE]->counter++;
        fillCanDataBuffer(MSG_MS_BASE, &canTemp);
        break;
      case MS_BASE_ID + 1:
        allCanMessages[MSG_MS_PLUS1]->counter++;
        fillCanDataBuffer(MSG_MS_PLUS1, &canTemp);
        break;
      case MS_BASE_ID + 2:
        allCanMessages[MSG_MS_PLUS2]->counter++;
        fillCanDataBuffer(MSG_MS_PLUS2, &canTemp);
        break;
      case MS_BASE_ID + 3:
        allCanMessages[MSG_MS_PLUS3]->counter++;
        fillCanDataBuffer(MSG_MS_PLUS3, &canTemp);
        break;
      case MS_BASE_ID + 4:
        allCanMessages[MSG_MS_PLUS4]->counter++;
        fillCanDataBuffer(MSG_MS_PLUS4, &canTemp);
        break;
      case CAN_SH_VSS_MSG_ID:
        if(USE_SENSORHUB_VSS)
        {
          sensorHubMph = ((canTemp.data[1] * 256) + canTemp.data[0]) / 10.0;
          
          union
          {
            long vss;
            byte buf[4];
          } vssUnion;
          
          vssUnion.buf[0] = canTemp.data[2];
          vssUnion.buf[1] = canTemp.data[3];
          vssUnion.buf[2] = canTemp.data[4];
          vssUnion.buf[3] = canTemp.data[5];
          vssPulseCAN = vssUnion.vss;
        }
        break;
      case CAN_SH_CLK_MSG_ID:
        datetimeCAN.hour     = canTemp.data[0];
        datetimeCAN.minute   = canTemp.data[1];
        datetimeCAN.second   = canTemp.data[2];
        datetimeCAN.month    = canTemp.data[3];
        datetimeCAN.day      = canTemp.data[4];
        datetimeCAN.year     = ((canTemp.data[6] * 256) + canTemp.data[5]);
        break;
      case CAN_SH_COMPASS_MSG_ID:
        compassDirectionIndexCAN = canTemp.data[2];
        break;
      default:
        canUnhandledCount++;
        break;
    }
  }
}

void fillCanDataBuffer(int index, canData* canTemp)
{
  for(int i = 0; i < 8; i++)
  {
    allCanMessages[index]->data[i] = canTemp->data[i];
  }
}

void setup() {
  lcd.begin(16, 2);

  lcd.createChar(charBlock1, fill1);
  lcd.createChar(charBlock2, fill2);
  lcd.createChar(charBlock3, fill3);
  lcd.createChar(charBlock4, fill4);
  lcd.createChar(charBlock5, fill5);
  lcd.createChar(charMiddle, fillMiddle);
  lcd.createChar(charBlank, fillNothing);

  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_SHIFT, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  clearBuffer(&canBufferTemp[0]);
  canTemp.data = &canBufferTemp[0];

  canBase.id = MS_BASE_ID;
  canBase.counter = 0;
  clearBuffer(&canBufferBase[0]);
  canBase.data = &canBufferBase[0];
  allCanMessages[0] = &canBase;

  canPlus1.id = MS_BASE_ID + 1;
  canPlus1.counter = 0;
  clearBuffer(&canBufferPlus1[0]);
  canPlus1.data = &canBufferPlus1[0];
  allCanMessages[1] = &canPlus1;
  
  canPlus2.id = MS_BASE_ID + 2;
  canPlus2.counter = 0;
  clearBuffer(&canBufferPlus2[0]);
  canPlus2.data = &canBufferPlus2[0];
  allCanMessages[2] = &canPlus2;
  
  canPlus3.id = MS_BASE_ID + 3;
  canPlus3.counter = 0;
  clearBuffer(&canBufferPlus3[0]);
  canPlus3.data = &canBufferPlus3[0];
  allCanMessages[3] = &canPlus3;

  canPlus4.id = MS_BASE_ID + 4;
  canPlus4.counter = 0;
  clearBuffer(&canBufferPlus4[0]);
  canPlus4.data = &canBufferPlus4[0];
  allCanMessages[4] = &canPlus4;

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
  allDisplays[16] = &display_060;
  allDisplays[17] = &display_14m;
  allDisplays[18] = &display_600;

  #pragma endregion

  lcd.print("Wait for ECU");
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

  //make sure watchdog timer is turned off
  //WDTCR = (0<<WDCE) | (0<<WDE) | (0 << WDP0) | (0 << WDP1);

  if(DEBUG) {
    Serial.println("CAN bus initialized");
  }

  clearBuffer(&canBuffer[0]);
  canMsg.cmd      = CMD_RX_DATA;
  canMsg.pt_data  = &canBuffer[0];
  canMsg.ctrl.ide = MESSAGE_PROTOCOL; 
  canMsg.id.std   = 0;
  canMsg.id.ext   = 0;
  canMsg.dlc      = MESSAGE_LENGTH;
  canMsg.ctrl.rtr = MESSAGE_RTR;

  while(can_cmd(&canMsg) != CAN_CMD_ACCEPTED);

  lcd.clear();

  //datasheet section 7.3 Watchdog Timer
  //enable watchdog timer (WDCE, WDE) and set timing (WDP0, WDP1)
  //WDTCR = (1<<WDCE) | (1<<WDE) | (1 << WDP0) | (1 << WDP1);

  //currentDisplayIndex = 17;



  //resetPerformanceTracker(tracker060);

  tracker14.isActive = false;
  tracker14.startMillis = 0;
  tracker14.endMillis = 0;
  tracker14.elapsedMillis = 0;
  tracker14.mph = 0.0;
  tracker14.startPulses = 0;
  tracker14.endPulses = 0;
  tracker14.elapsedPulses = 0;

  tracker060.isActive = false;
  tracker060.startMillis = 0;
  tracker060.endMillis = 0;
  tracker060.elapsedMillis = 0;
  tracker060.mph = 0.0;
  tracker060.startPulses = 0;
  tracker060.endPulses = 0;
  tracker060.elapsedPulses = 0;

//   t->isActive = false;
//   t->startMillis = 0;
//   t->endMillis = 0;
//   t->elapsedMillis = 0;
//   t->mph = 0.0;
//   t->startPulses = 0;
//   t->endPulses = 0;
//   t->elapsedPulses = 0;



}

bool canCheckComplete = false;

void loop() {
  noInterrupts();
  processCanMessages();
  interrupts();
  Display* d = allDisplays[currentDisplayIndex];

  //calculate 0-60
  if(d->type == zerotosixty)
  {
    if(tracker060.isActive)
    {
      tracker060.elapsedMillis = millis() - tracker060.startMillis;
      tracker060.mph = engine_vss.currentValue;     
    }
    if(tracker060.startMillis == 0 && engine_vss.currentValue > 0.0 && engine_vss.previousValue == 0.0)
    {
      tracker060.startMillis = millis();
      tracker060.isActive = true;
      tracker060.mph = engine_vss.currentValue;
    }
    if(tracker060.isActive && engine_vss.currentValue >= 60.0)
    {
      tracker060.endMillis = millis();
      tracker060.isActive = false;
      tracker060.mph = engine_vss.currentValue;
      startShortBuzzer();
    }
  }

  if(d->type == sixtytozero)
  {
    tracker600.mph = engine_vss.currentValue;

    if(tracker600.isActive)
    {
      tracker600.elapsedMillis = millis() - tracker600.startMillis;
    }
    if(tracker600.startMillis == 0 && engine_vss.currentValue < 60.0 && engine_vss.previousValue >= 60.0)
    {
      tracker600.startMillis = millis();
      tracker600.isActive = true;
    }
    if(tracker600.isActive && engine_vss.currentValue == 0.0)
    {
      tracker600.endMillis = millis();
      tracker600.isActive = false;
      startShortBuzzer();
    }
  }



  if(d->type == quartermile)
  {

    if(tracker14.isActive)
    {
      tracker14.elapsedMillis = millis() - tracker14.startMillis;
      tracker14.elapsedPulses = vssPulseSafe - tracker14.startPulses;
      tracker14.mph = engine_vss.currentValue;
    }

    //get started
    if(tracker14.startMillis == 0 && engine_vss.currentValue > 0.0 && engine_vss.previousValue == 0.0)
    {
      tracker14.elapsedMillis = 0;
      tracker14.startMillis = millis();
      tracker14.startPulses = vssPulseSafe;
      tracker14.isActive = true;
      //calculate 1/4 mile time
    }

    //finish
    if(tracker14.isActive && tracker14.elapsedPulses >= (VSS_PULSE_PER_MILE / 4))
    {
      tracker14.endMillis = millis();
      tracker14.mph = engine_vss.currentValue;
      tracker14.isActive = false;
      startShortBuzzer();
    }


    
    
  }


  if(DEBUG)
  {
    char* formattedRuntime = formatRuntime(millis());
    Serial.println(formattedRuntime);
  }

  if(DEBUG_CAN)
  {
    Serial.print(allCanMessages[MSG_MS_BASE]->counter);
    Serial.print(",");
    Serial.print(allCanMessages[MSG_MS_PLUS1]->counter);
    Serial.print(",");
    Serial.print(allCanMessages[MSG_MS_PLUS2]->counter);
    Serial.print(",");
    Serial.print(allCanMessages[MSG_MS_PLUS3]->counter);
    Serial.print(",");
    Serial.print(allCanMessages[MSG_MS_PLUS4]->counter);
    Serial.print(",         ");
    Serial.println(canCount);
  }

  if(DEBUG_ENG)
  {
    int gaugeCount = sizeof(allGauges) / sizeof(allGauges[0]);
    for(int i = 0; i < gaugeCount; i++)
    {
      Serial.print(allGauges[i]->shortLabel);
      Serial.print(": ");
      Serial.println(allGauges[i]->currentValue);
    }
    Serial.println("=====");
  }

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
      if(USE_NOISY_BUTTON)
      {
        startClickBuzzer();
      }
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
          lcd.clear();
          for(int i = 0; i < 16; i++)
          {
            lcd.write(charBlock5);
          }
          lcd.setCursor(0,1);
          for(int i = 0; i < 16; i++)
          {
            lcd.write(charBlock5);
          }
          delay(1000);
          resetFunc();
       }
    }
  }

  if(!canCheckComplete && currentMillis > 2000)
  {
    if(DEBUG)
    {
      Serial.println("CAN CHECK---------------------------------------------");
    }
    bool mustReset = false;
    int length = sizeof(allCanMessages) / sizeof(allCanMessages[0]);
    for(int i = 0; i < length; i++)
    {
      if(allCanMessages[i]->counter < 20)
      {
        mustReset = true;
        //WDTCR = (1<<WDCE) | (1<<WDE) | (0 << WDP0) | (0 << WDP1);
        //delay(100);
      }
    }
    if(mustReset)
    {
      if(DEBUG)
      {
        Serial.println("MUST RESET---------------------------------------------");
      }
    }
    canCheckComplete = true;
  }

  if(currentMillis - shortBuzzerStartMillis >= shortBuzzerInterval && shortBuzzerStartMillis > 0)
  {
    digitalWrite(BUZZER_PIN, LOW);
    shortBuzzerStartMillis = 0;
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
        if(err == true)
        {
          startShortBuzzer();
        }
        Serial.println("ERROR============================================================================");
      }
    }
  }
}

void startShortBuzzer()
{
  digitalWrite(BUZZER_PIN, HIGH);
  shortBuzzerStartMillis = millis();
}

void startClickBuzzer()
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(1);
  digitalWrite(BUZZER_PIN, LOW);
}

char* formatRuntime(uint32_t milliseconds)
{
  uint32_t seconds = milliseconds / 1000;
  int runHours = seconds / 3600;
  int secsRemaining = seconds % 3600;
  int runMinutes = secsRemaining / 60;
  int runSeconds = secsRemaining % 60;

  if(runHours > 0)
  {
    char buf[9];
    sprintf(buf,"%02d:%02d:%02d",runHours,runMinutes,runSeconds);
    char* ret = buf;
    return ret;
  }
  else
  {
    char buf[6];
    sprintf(buf,"%02d:%02d",runMinutes,runSeconds);
    char* ret = buf;
    return ret;
  }
}

double getTripOdometer()
{
  return (double)vssPulseSafe / (double)VSS_PULSE_PER_MILE;
}

void nextDisplay() 
{
  if(DEBUG)
  {
    Serial.println("next display");
  }
  //clearDisplay();
  lcd.clear();
  currentDisplayIndex++;
  if(currentDisplayIndex > (DISPLAY_COUNT - 1))
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
  else if(d->type == zerotosixty)
  {
    drawZeroToSixty();
  }
  else if(d->type == quartermile)
  {
    drawQuarterMile();
  }
  else if(d->type == sixtytozero)
  {
    drawSixtyToZero();
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
    lcd.setCursor(0, 0);
    lcd.print("Warmup");
    lcd.setCursor(6, 0);

    static char outstr[15];
    dtostrf(engine_clt.currentValue,4, engine_clt.decimalPlaces, outstr);
    lcd.print(outstr);
    lcd.print((char)223); //degree symbol

    drawBar(startupCLT, 160.0, engine_clt.currentValue, 1, 0, 16);
  }
}

void drawRuntime()
{
  if(USE_SENSORHUB_GPS)
  {
    lcd.setCursor(0,0);
    if(datetimeSafe.month < 1)
    {
      lcd.print("No GPS");
    }
    else
    {
      char formattedTime[7];
      uint8_t hour = datetimeSafe.hour;
      char* ampm = "   ";       //ampm is 3 chars to cover up entire "No GPS" string
      if(!USE_24HOUR_TIME)
      {
        if(hour > 12)
        {
          hour = hour - 12;
          ampm = "pm ";         //ampm is 3 chars to cover up entire "No GPS" string
        }
        else
        {
          ampm = "am ";         //ampm is 3 chars to cover up entire "No GPS" string
        }
      }
      sprintf(formattedTime, "%02d:%02d", hour, datetimeSafe.minute);
      lcd.print(formattedTime);
      lcd.print(ampm);
    }
    if(USE_SENSORHUB_COMPASS)
    {
      lcd.setCursor(14,0);
      lcd.print(compassDirections[compassDirectionIndexSafe]);
    }
    lcd.setCursor(0,1);
    char* formattedRuntime = formatRuntime(millis());
    lcd.print(formattedRuntime);
    lcd.setCursor(9,1);
    double trip = getTripOdometer();
    if(trip >= 1000.0)
    {
      char str_temp[5];
      dtostrf(trip, 5, 0, str_temp);
      lcd.print(str_temp);
    }
    else if(trip >= 100.0)
    {
      char str_temp[5];
      dtostrf(trip, 5, 1, str_temp);
      lcd.print(str_temp);
    }
    else if(trip >= 10.0)
    {
      char str_temp[5];
      dtostrf(trip, 5, 2, str_temp);
      lcd.print(str_temp);
    }
    else
    {
      char str_temp[5];
      dtostrf(trip, 5, 3, str_temp);
      lcd.print(str_temp);
    }
    lcd.print("mi");
  }
  else
  {
    lcd.setCursor(0,0);
    lcd.print("Runtime");

    char* formattedRuntime = formatRuntime(millis());
    lcd.setCursor(8,0);
    lcd.print(formattedRuntime);
  }
}

void drawZeroToSixty()
{
  lcd.setCursor(0, 0);
  lcd.print("0-60 ");


  if(tracker060.isActive && tracker060.mph == 0.0 && tracker060.elapsedMillis > 2000)
  {
    tracker060.isActive = false;
  }


  char vssBuffer[2];
  sprintf(vssBuffer,"%02d",(uint16_t)tracker060.mph);
  lcd.print(vssBuffer);

  lcd.print("mph");

  if(tracker060.mph == 0.0)
  {
    tracker060.startMillis = 0;
    tracker060.elapsedMillis = 0;
    tracker060.endMillis = 0;
    tracker060.isActive = false;
    lcd.setCursor(0,1);
    lcd.print("READY!          ");

    lcd.setCursor(11, 0);
    lcd.print("     ");

  }
  else 
  {
    if(tracker060.isActive)
    {
      lcd.setCursor(11, 0);

      double elapsedSeconds = (double)tracker060.elapsedMillis / 1000.0;
      sprintf(vssBuffer,"%02d",(uint16_t)engine_vss.currentValue);
      lcd.print(elapsedSeconds,1);
      drawBar(0.0, 60.0, engine_vss.currentValue, 1, 0, 16);
    }
    else
    {
      if(engine_vss.currentValue == 0.0)
      {
        tracker060.startMillis = 0;
        tracker060.elapsedMillis = 0;
        tracker060.endMillis = 0;
        tracker060.mph = 0;
        tracker060.startPulses = 0;
        tracker060.endPulses = 0;
        tracker060.elapsedPulses = 0;
        // lcd.setCursor(0,1);
        // lcd.print("READY!          ");
      }
    }

  }
}

void drawQuarterMile()
{
  lcd.setCursor(0, 0);
  lcd.print("1/4mi");

  lcd.setCursor(6, 0);

  if(tracker14.isActive && tracker14.mph == 0.0 && tracker14.elapsedMillis > 2000)
  {
    tracker14.isActive = false;
  }

  double elapsedSeconds = (double)tracker14.elapsedMillis / 1000.0;
  lcd.print(elapsedSeconds,1);
  lcd.print(" @ ");

  char vssBuffer[3];
  sprintf(vssBuffer,"%03d",round(tracker14.mph));
  lcd.print(vssBuffer);
  lcd.print(" ");

  if(tracker14.isActive)
  {
    drawBar(0, (VSS_PULSE_PER_MILE / 4), tracker14.elapsedPulses, 1, 0, 16);
  }
  else
  {
    if(engine_vss.currentValue == 0.0)
    {
      tracker14.startMillis = 0;
      tracker14.elapsedMillis = 0;
      tracker14.endMillis = 0;
      tracker14.mph = 0;
      tracker14.startPulses = 0;
      tracker14.endPulses = 0;
      tracker14.elapsedPulses = 0;
      lcd.setCursor(0,1);
      lcd.print("READY!          ");
    }
  }
}

void drawSixtyToZero()
{
  lcd.setCursor(0, 0);
  lcd.print("60-0 ");

  if(tracker600.isActive && tracker600.mph >= 60.0)
  {
    tracker600.isActive = false;
  }


  char vssBuffer[2];
  sprintf(vssBuffer,"%02d",(uint16_t)tracker600.mph);
  lcd.print(vssBuffer);

  lcd.print("mph");

  if(tracker600.mph >= 60.0)
  {
    tracker600.startMillis = 0;
    tracker600.elapsedMillis = 0;
    tracker600.endMillis = 0;
    tracker600.isActive = false;
    lcd.setCursor(0,1);
    lcd.print("READY!          ");

    lcd.setCursor(11, 0);
    lcd.print("     ");

  }
  else 
  {
    if(tracker600.isActive)
    {
      lcd.setCursor(11, 0);

      double elapsedSeconds = (double)tracker600.elapsedMillis / 1000.0;
      sprintf(vssBuffer,"%02d",(uint16_t)engine_vss.currentValue);
      lcd.print(elapsedSeconds,1);
      drawBar(60.0, 0.0, engine_vss.currentValue, 1, 0, 16);
    }
    else
    {
      if(engine_vss.currentValue >= 60.0)
      {
        tracker600.startMillis = 0;
        tracker600.elapsedMillis = 0;
        tracker600.endMillis = 0;
        tracker600.mph = 0;
        tracker600.startPulses = 0;
        tracker600.endPulses = 0;
        tracker600.elapsedPulses = 0;
        // lcd.setCursor(0,1);
        // lcd.print("READY!          ");
      }
    }

  }
}

void processCanMessages()
{
  if(engine_map.canCounter < allCanMessages[MSG_MS_BASE]->counter)
  {
    engine_map.previousValue = engine_map.currentValue;
    engine_map.currentValue = ((allCanMessages[MSG_MS_BASE]->data[0] * 256) + allCanMessages[MSG_MS_BASE]->data[1]) / 10.0;
    engine_map.canCounter = allCanMessages[MSG_MS_BASE]->counter;
    incrementQualityCounters(&engine_map);
  }

  if(engine_rpm.canCounter < allCanMessages[MSG_MS_BASE]->counter)
  {
    engine_rpm.previousValue = engine_rpm.currentValue;
    //engine_rpm.currentValue = buf[2] * 256 + buf[3];
    //round rpm to nearest 10
    engine_rpm.currentValue = round((allCanMessages[MSG_MS_BASE]->data[2] * 256 + allCanMessages[MSG_MS_BASE]->data[3]) / 10.0) * 10.0;
    engine_rpm.canCounter = allCanMessages[MSG_MS_BASE]->counter;
    incrementQualityCounters(&engine_rpm);
  }
  
  if(engine_clt.canCounter < allCanMessages[MSG_MS_BASE]->counter)
  {
    engine_clt.previousValue = engine_clt.currentValue;
    engine_clt.currentValue = (allCanMessages[MSG_MS_BASE]->data[4] * 256 + allCanMessages[MSG_MS_BASE]->data[5]) / 10.0;
    if(startupCLT == STARTUP_CLT_VALUE)
    {
      startupCLT = engine_clt.currentValue;
    }
    engine_clt.canCounter = allCanMessages[MSG_MS_BASE]->counter;
    incrementQualityCounters(&engine_clt);
  }

  if(engine_tps.canCounter < allCanMessages[MSG_MS_BASE]->counter)
  {
    engine_tps.previousValue = engine_tps.currentValue;
    engine_tps.currentValue = (allCanMessages[MSG_MS_BASE]->data[6] * 256 + allCanMessages[MSG_MS_BASE]->data[7]) / 10.0;
    engine_tps.canCounter = allCanMessages[MSG_MS_BASE]->counter;
    incrementQualityCounters(&engine_tps);
  }

  ////////////////////        

  if(engine_pw1.canCounter < allCanMessages[MSG_MS_PLUS1]->counter)
  {
    engine_pw1.previousValue = engine_pw1.currentValue;
    engine_pw1.currentValue = (allCanMessages[MSG_MS_PLUS1]->data[0] * 256 + allCanMessages[MSG_MS_PLUS1]->data[1]) / 1000.0;
    engine_pw1.canCounter = allCanMessages[MSG_MS_PLUS1]->counter;
    incrementQualityCounters(&engine_pw1);
  }

  if(engine_pw2.canCounter < allCanMessages[MSG_MS_PLUS1]->counter)
  {
    engine_pw2.previousValue = engine_pw2.currentValue;
    engine_pw2.currentValue = (allCanMessages[MSG_MS_PLUS1]->data[2] * 256 + allCanMessages[MSG_MS_PLUS1]->data[3]) / 1000.0;
    engine_pw2.canCounter = allCanMessages[MSG_MS_PLUS1]->counter;
    incrementQualityCounters(&engine_pw2);
  }

  if(engine_iat.canCounter < allCanMessages[MSG_MS_PLUS1]->counter)
  {
    engine_iat.previousValue = engine_iat.currentValue;
    engine_iat.currentValue = (allCanMessages[MSG_MS_PLUS1]->data[4] * 256 + allCanMessages[MSG_MS_PLUS1]->data[5]) / 10.0;
    engine_iat.canCounter = allCanMessages[MSG_MS_PLUS1]->counter;
    incrementQualityCounters(&engine_iat);
  }

  if(engine_adv.canCounter < allCanMessages[MSG_MS_PLUS1]->counter)
  {
    engine_adv.previousValue = engine_adv.currentValue;
    engine_adv.currentValue = (allCanMessages[MSG_MS_PLUS1]->data[6] * 256 + allCanMessages[MSG_MS_PLUS1]->data[7]) / 10.0;
    engine_adv.canCounter = allCanMessages[MSG_MS_PLUS1]->counter;
    incrementQualityCounters(&engine_adv);
  }

  ////////////////////

  if(engine_tgt.canCounter < allCanMessages[MSG_MS_PLUS2]->counter)
  {
    engine_tgt.previousValue = engine_tgt.currentValue;
    engine_tgt.currentValue = (double)allCanMessages[MSG_MS_PLUS2]->data[0] / 10.0;
    engine_tgt.canCounter = allCanMessages[MSG_MS_PLUS2]->counter;
    incrementQualityCounters(&engine_tgt);
  }     

  if(engine_afr.canCounter < allCanMessages[MSG_MS_PLUS2]->counter)
  {
    engine_afr.previousValue = engine_afr.currentValue;
    engine_afr.currentValue = (double)allCanMessages[MSG_MS_PLUS2]->data[1] / 10.0;
    engine_afr.canCounter = allCanMessages[MSG_MS_PLUS2]->counter;
    incrementQualityCounters(&engine_afr);
  }

  if(engine_ego.canCounter < allCanMessages[MSG_MS_PLUS2]->counter)
  {
    engine_ego.previousValue = engine_ego.currentValue;
    engine_ego.currentValue = (allCanMessages[MSG_MS_PLUS2]->data[2] * 256 + allCanMessages[MSG_MS_PLUS2]->data[3]) / 10.0;
    engine_ego.canCounter = allCanMessages[MSG_MS_PLUS2]->counter;
    incrementQualityCounters(&engine_ego);
  }

  if(engine_egt.canCounter < allCanMessages[MSG_MS_PLUS2]->counter)
  {
    engine_egt.previousValue = engine_egt.currentValue;
    engine_egt.currentValue = (allCanMessages[MSG_MS_PLUS2]->data[4] * 256 + allCanMessages[MSG_MS_PLUS2]->data[5]) / 10.0;
    engine_egt.canCounter = allCanMessages[MSG_MS_PLUS2]->counter;
    incrementQualityCounters(&engine_egt);
  }

  if(engine_pws.canCounter < allCanMessages[MSG_MS_PLUS2]->counter)
  {
    engine_pws.previousValue = engine_pws.currentValue;
    engine_pws.currentValue = (allCanMessages[MSG_MS_PLUS2]->data[6] * 256 + allCanMessages[MSG_MS_PLUS2]->data[7]) / 1000.0;
    engine_pws.canCounter = allCanMessages[MSG_MS_PLUS2]->counter;
    incrementQualityCounters(&engine_pws);
  }

  ////////////////////

  if(engine_bat.canCounter < allCanMessages[MSG_MS_PLUS2]->counter)
  {
    engine_bat.previousValue = engine_bat.currentValue;
    engine_bat.currentValue = (allCanMessages[MSG_MS_PLUS3]->data[0] * 256 + allCanMessages[MSG_MS_PLUS3]->data[1]) / 10.0;
    engine_bat.canCounter = allCanMessages[MSG_MS_PLUS2]->counter;
    incrementQualityCounters(&engine_bat);
  }

  if(engine_sr1.canCounter < allCanMessages[MSG_MS_PLUS3]->counter)
  {
    //not tested
    engine_sr1.previousValue = engine_sr1.currentValue;
    engine_sr1.currentValue = (allCanMessages[MSG_MS_PLUS3]->data[2] * 256 + allCanMessages[MSG_MS_PLUS3]->data[3]) / 10.0;
    engine_sr1.canCounter = allCanMessages[MSG_MS_PLUS3]->counter;
    incrementQualityCounters(&engine_sr1);
  }

  if(engine_sr2.canCounter < allCanMessages[MSG_MS_PLUS3]->counter)
  {
    //not tested
    engine_sr2.previousValue = engine_sr2.currentValue;
    engine_sr2.currentValue = (allCanMessages[MSG_MS_PLUS3]->data[4] * 256 + allCanMessages[MSG_MS_PLUS3]->data[5]) / 10.0;
    engine_sr2.canCounter = allCanMessages[MSG_MS_PLUS3]->counter;
    incrementQualityCounters(&engine_sr2);
  }

  if(engine_knk.canCounter < allCanMessages[MSG_MS_PLUS3]->counter)
  {
    //not tested
    engine_knk.previousValue = engine_knk.currentValue;
    engine_knk.currentValue = (allCanMessages[MSG_MS_PLUS3]->data[6] * 256) / 10.0;
    engine_knk.canCounter = allCanMessages[MSG_MS_PLUS3]->counter;
    incrementQualityCounters(&engine_knk);
  }

  ////////////////////

  if(USE_SENSORHUB_VSS)
  {
    engine_vss.previousValue = engine_vss.currentValue;
    engine_vss.currentValue = sensorHubMph;
    incrementQualityCounters(&engine_vss);

    vssPulseSafe = vssPulseCAN;
  }
  else
  {
    if(engine_vss.canCounter < allCanMessages[MSG_MS_PLUS4]->counter)
    {
      engine_vss.previousValue = engine_vss.currentValue;
      engine_vss.currentValue = (allCanMessages[MSG_MS_PLUS4]->data[0] * 256 + allCanMessages[MSG_MS_PLUS4]->data[1]) / 10.0;
      engine_vss.canCounter = allCanMessages[MSG_MS_PLUS4]->counter;
      incrementQualityCounters(&engine_vss);
    }
  }
  
  if(engine_tcr.canCounter < allCanMessages[MSG_MS_PLUS4]->counter)
  {
    //not tested
    engine_tcr.previousValue = engine_tcr.currentValue;
    engine_tcr.currentValue = (allCanMessages[MSG_MS_PLUS4]->data[2] * 256 + allCanMessages[MSG_MS_PLUS4]->data[3]) / 10.0;
    engine_tcr.canCounter = allCanMessages[MSG_MS_PLUS4]->counter;
    incrementQualityCounters(&engine_tcr);
  }

  if(engine_lct.canCounter < allCanMessages[MSG_MS_PLUS4]->counter)
  {
    engine_lct.previousValue = engine_lct.currentValue;
    engine_lct.previousValue = (allCanMessages[MSG_MS_PLUS4]->data[4] * 256 + allCanMessages[MSG_MS_PLUS4]->data[5]) / 10.0;
    engine_lct.canCounter = allCanMessages[MSG_MS_PLUS4]->counter;
    incrementQualityCounters(&engine_lct);
  }

  if(USE_SENSORHUB_GPS)
  {
    //cannot copy directly, doing it manually is clunky but maybe less brittle than memcpy
    datetimeSafe.hour     = datetimeCAN.hour;
    datetimeSafe.minute   = datetimeCAN.minute;
    datetimeSafe.second   = datetimeCAN.second;
    datetimeSafe.month    = datetimeCAN.month;
    datetimeSafe.day      = datetimeCAN.day;
    datetimeSafe.year     = datetimeCAN.year;
  }

  if(USE_SENSORHUB_COMPASS)
  {
    compassDirectionIndexSafe = compassDirectionIndexCAN;
  }

  if(DEBUG_ENG && false)
  {
    for(int i = 0; i < 8; i++)
    {
      Serial.print(allCanMessages[MSG_MS_BASE]->data[i]);
      Serial.print(" ");
    }
    Serial.println("");
    Serial.println("~~~~~");

    for(int i = 0; i < 8; i++)
    {
      Serial.print(allCanMessages[MSG_MS_PLUS1]->data[i]);
      Serial.print(" ");
    }
    Serial.println("");
    Serial.println("~~~~~");

    for(int i = 0; i < 8; i++)
    {
      Serial.print(allCanMessages[MSG_MS_PLUS2]->data[i]);
      Serial.print(" ");
    }
    Serial.println("");
    Serial.println("~~~~~");

    for(int i = 0; i < 8; i++)
    {
      Serial.print(allCanMessages[MSG_MS_PLUS3]->data[i]);
      Serial.print(" ");
    }
    Serial.println("");
    Serial.println("~~~~~");

    for(int i = 0; i < 8; i++)
    {
      Serial.print(allCanMessages[MSG_MS_PLUS4]->data[i]);
      Serial.print(" ");
    }
    Serial.println("");
    Serial.println("~~~~~");
  }
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

void incrementQualityCounters(EngineVariable* engine) {
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
  lcd.setCursor(column-1, row-1);
  int length = map(engineVar->currentValue, engineVar->minimum, engineVar->maximum, 0, maxLength);

  for(int i = 0; i < maxLength; i++) {
    if(i > length) {
      lcd.setCursor(column-1+i, row-1);
      lcd.write((uint8_t)charBlank);
    }
    else {
      lcd.setCursor(column-1+i, row-1);
      lcd.write((uint8_t)charBlock5);
    }
  }
}

void drawBar(float lowValue, float highValue, float currentValue, int row, int column, int maxLength)
{
  int length = map(currentValue, lowValue, highValue, 0, maxLength);
  for(int i = 0; i < maxLength; i++) {
    if(i > length) {
      lcd.setCursor(column+i, row);
      lcd.write(charBlank);
    }
    else {
      lcd.setCursor(column+i, row);
      lcd.write(charBlock5);
    }
  }
}

bool calculateErrorLight() {
  uint8_t len = 20; //sizeof(allGauges);
  uint32_t badCount;
  uint32_t totalCount;
  uint8_t percent;
  bool inError = false;
  
  for(uint8_t i = 0; i < len; i++) {
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
    startShortBuzzer();
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
  char* gaugeLabel = gauge->longLabel;
  float gaugeValue = gauge->currentValue;
  int gaugeDecimal = gauge->decimalPlaces;
  lcd.setCursor(0,0);
  lcd.print(gaugeLabel);
  lcd.setCursor(10,0);
  static char outstr[15];
  dtostrf(gaugeValue,5, gaugeDecimal, outstr);
  lcd.print(outstr);
  drawBar(gauge, 2, 1, 16);
}

void drawDualGauge(EngineVariable* gauge1, EngineVariable* gauge2)
{
  char* gauge1Label = gauge1->shortLabel;
  float gauge1Value = gauge1->currentValue;
  int gauge1Decimal = gauge1->decimalPlaces;
  lcd.setCursor(0,0);
  lcd.print(gauge1Label);

  lcd.setCursor(4,0);
  static char outstr1[15];
  dtostrf(gauge1Value,4, gauge1Decimal, outstr1);
  lcd.print(outstr1);
  
  drawBar(gauge1, 1, 9, 8);

  char* gauge2Label = gauge2->shortLabel;
  float gauge2Value = gauge2->currentValue;
  int gauge2Decimal = gauge2->decimalPlaces;
  
  lcd.setCursor(0, 1);
  lcd.print(gauge2Label);
  
  lcd.setCursor(4,1);
  static char outstr2[15];
  dtostrf(gauge2Value,4, gauge2Decimal, outstr2);
  lcd.print(outstr2);
  
  drawBar(gauge2, 2, 9, 8);
}
