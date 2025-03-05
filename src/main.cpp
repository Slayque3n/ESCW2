#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include "Knob.h"
#include <algorithm>
#include <ES_CAN.h>

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

//Instantiate the message

//Step Sizes
const uint32_t stepSizes [] = {51076056,54113197,57330935,60740010,64351798,68178356,72232452,76527617,81078186,85899345,91007186, 96418755};
const std::string notes[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};


volatile uint32_t currentStepSize;

//waveforms
const unsigned char sinetable[128] = {
  0,   0,   0,   0,   1,   1,   1,   2,   2,   3,   4,   5,   5,   6,   7,   9,
 10,  11,  12,  14,  15,  17,  18,  20,  21,  23,  25,  27,  29,  31,  33,  35,
 37,  40,  42,  44,  47,  49,  52,  54,  57,  59,  62,  65,  67,  70,  73,  76,
 79,  82,  85,  88,  90,  93,  97, 100, 103, 106, 109, 112, 115, 118, 121, 124,
128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173,
176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215,
218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244,
245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
};


//Interupt timer
HardwareTimer sampleTimer(TIM1);

//Queue handler 
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;


SemaphoreHandle_t CAN_TX_Semaphore;

//Global system state struct
struct {
  uint8_t RX_Message[8];
  std::bitset<32> inputs;
  Knob knob0{0, 8, 8, 0};
  Knob volume{1, 0, 8, 0};
  Knob octave{2, 1, 7, 4};
  Knob waveform{3, 0, 3, 0};
  SemaphoreHandle_t mutex;
} sysState;

Knob* knobs[] = {&sysState.knob0, &sysState.waveform, &sysState.octave, &sysState.volume};

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

//Function to read inputs from switch matrix columns
std::bitset<4> readCols(){
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;

}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

void setISR(){
  static uint32_t phaseAcc = 0;
  int32_t Vout = 0;
  int k3r = __atomic_load_n(&sysState.volume.rotation, __ATOMIC_ACQUIRE); //volume
  int k2r = __atomic_load_n(&sysState.octave.rotation, __ATOMIC_ACQUIRE); //octave
  int k1r = __atomic_load_n(&sysState.waveform.rotation, __ATOMIC_ACQUIRE); //waveform


  phaseAcc += currentStepSize;


  //adjust the wavetype
  if(k1r == 0){
    // sawtooth waveform
    Vout = (phaseAcc >> 24) - 128;
  }
  else if(k1r == 1) {
    // sqaure waveform
    Vout = (phaseAcc >> 24) > 128 ? -128 : 127;
  }
  else if(k1r == 2){
    if ((phaseAcc >> 24) >= 128) {
      Vout = 2*(((255 - (phaseAcc >> 24)) * 2) - 127);
    }
    else {
      Vout = 2*((phaseAcc >> 23) - 128);
    }
  }
  else if(k1r == 3){
    int i;

    if ((phaseAcc >> 24) >= 128) {
      i = 255 - (phaseAcc >> 24);
    }
    else {
      i = phaseAcc >> 24;
    }

    Vout = (sinetable[i] - 128);
  }
  //adjust the volume
  Vout = Vout >> (8 - k3r);

  analogWrite(OUTR_PIN, Vout + 128);
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}


void scanKeysTask(void * pvParameters) {
  
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static int lastIncrement;
  std::bitset<4> cols;
  std::bitset<32> previousInputs;
  uint8_t TX_Message[8] = {0};


  for(;;){ 
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    previousInputs = sysState.inputs;

    for(int i=0;i<5;i++){
      setRow(i);
      delayMicroseconds(3);
      cols = readCols();
      for (int j = 0; j < 4; j++) sysState.inputs[4*i + j] = cols[j];
    }

    for (int i = 0; i < 12; i++){
      if (sysState.inputs[i] != previousInputs[i]){
        TX_Message[0] = (sysState.inputs[i] & 0b1) ? 'R' : 'P';
        TX_Message[1] = sysState.octave.rotation;
        TX_Message[2] = i;
        xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);      }
    }

    for(int i=0;i<4;i++){
      int currentStateA = sysState.inputs[(12 + (3-i)*2)];
      int currentStateB = sysState.inputs[(13 + (3-i)*2)];
      knobs[i]->updateValues(currentStateA, currentStateB);
    }
    xSemaphoreGive(sysState.mutex);
    //__atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }  
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID;
  uint8_t local_RX_Message[8] = {0};
  

  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

      //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    //u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    char cstr0[2], cstr1[2], cstr2[2], cstr3[2];
    snprintf(cstr0, sizeof(cstr0), "%d", sysState.knob0.rotation);
    snprintf(cstr1, sizeof(cstr1), "%d", sysState.waveform.rotation);
    snprintf(cstr2, sizeof(cstr2), "%d", sysState.octave.rotation);
    snprintf(cstr3, sizeof(cstr3), "%d", sysState.volume.rotation);

    u8g2.drawStr(2,10, cstr0);
    u8g2.drawStr(20,10, cstr1);
    u8g2.drawStr(38,10, cstr2);
    u8g2.drawStr(56,10, cstr3);  

    u8g2.setCursor(66,30);
    u8g2.print((char) sysState.RX_Message[0]);
    u8g2.print(sysState.RX_Message[1]);
    u8g2.print(sysState.RX_Message[2]);

    u8g2.setCursor(2,20);
    u8g2.print(sysState.inputs.to_ulong(),HEX);
    xSemaphoreGive(sysState.mutex);
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask (void * pvParameters) {
  uint32_t localCurrentStepSize = 0;
  uint8_t local_octave;
  uint8_t local_RX_Message[8] = {0};

  while (1) {
    xQueueReceive(msgInQ, local_RX_Message, portMAX_DELAY);
    if (local_RX_Message [0] == 'P') {
      if(local_RX_Message[1] > 3){
        localCurrentStepSize = stepSizes[local_RX_Message[2]] << (local_RX_Message[1] - 4);
      }
      else{
        localCurrentStepSize = stepSizes[local_RX_Message[2]] >> (4 - local_RX_Message[1]);
      }
    }
    else {        
      localCurrentStepSize = 0;
    } 
    
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);

      for (int i = 0; i < 8; i++) {
        sysState.RX_Message[i] = local_RX_Message[i];
      }
    xSemaphoreGive(sysState.mutex);
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }  
}

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void setup() {
  // put your setup code here, to run once:
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  CAN_Init(true);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  setCANFilter(0x123,0x7ff);
  CAN_Start();
  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(setISR);
  sampleTimer.resume();
  sysState.mutex = xSemaphoreCreateMutex();
  TaskHandle_t scanKeysHandle = NULL;

  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &scanKeysHandle);	/* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdateHandle);	/* Pointer to store the task handle */

  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decode",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &decodeHandle);	/* Pointer to store the task handle */
  
  TaskHandle_t CAN_TX_Handle = NULL;
  xTaskCreate(
  CAN_TX_Task,		/* Function that implements the task */
  "CAN_TX",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &CAN_TX_Handle);	/* Pointer to store the task handle */

  vTaskStartScheduler();
}

void loop() {
}