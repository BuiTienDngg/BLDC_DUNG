#include <Arduino.h>
#define EN 8
#define pwmA 9
#define pwmB 5
#define pwmC 6
#define en digitalWrite(EN,HIGH)
#define offen digitalWrite(EN,LOW)
volatile uint8_t speed =  50;
const int pwmFrequency = 20000; // Tần số PWM, 20 kHz
const int pwmResolution = 8;    // Độ phân giải PWM, 8-bit
int phaseStates[6][3] = {
    {HIGH, LOW, LOW},  // Pha 1
    {HIGH, HIGH, LOW}, // Pha 2
    {LOW, HIGH, LOW},  // Pha 3
    {LOW, HIGH, HIGH}, // Pha 4
    {LOW, LOW, HIGH},  // Pha 5
    {HIGH, LOW, HIGH}  // Pha 6
};
int stepCounter = 0;
const uint16_t HallU = 19;
const uint16_t HallV = 20;
const uint16_t HallW = 21;
volatile uint8_t stateU;
volatile uint8_t stateV;
volatile uint8_t stateW;
uint8_t count[6] = {0};
uint8_t currentPos = 0;
uint8_t nextPos = 0;
uint8_t numberofCode = 0;
uint8_t i = 0;
uint8_t cnt = 0;
uint8_t start = 0;
uint8_t statePhaseHall[6] = {4,5,1,3,2,6}; //100,101,001,011,010,110
uint8_t updateHall(){
  numberofCode = stateU << 2 | stateV << 1 | stateW ;
  return numberofCode;
}
void printEncoder(uint8_t code){
  uint8_t tmp1,  tmp2, tmp3, tmp4 = 0;
  tmp1 = (code >> 2) & 0x01;
  tmp2 = (code >> 1) & 0x01;
  tmp3 = (code >> 0) & 0x01;
  
  Serial.print(tmp1);
  Serial.print(tmp2);
  Serial.print(tmp3);
  Serial.print("   ");
  Serial.print(tmp4);
  Serial.print("i = ");
  Serial.print(i);
  Serial.println();
}
void isrU() {
  stateU = digitalRead(HallU);
  // stateU ++;
}
void isrV() {
  stateV = digitalRead(HallV);
  // stateV++;
}
void isrW() {
  stateW = digitalRead(HallW);
}
void logHallinterrupt(){
  char data[20];
  sprintf(data,"U=%d,V=%d,W=%d", stateU,stateV,stateW);
  Serial.println(data);
}

uint8_t isPreviousPos(uint8_t currentPos, uint8_t lastPos){
// 100 <- 101 <- 001 <- 011 <- 010 <- 110 
// lùi
  uint8_t indexCurrentPos = 0;
  uint8_t indexLastPos = 0;
  for(int i = 0; i < 6; i++){
    if(currentPos == statePhaseHall[i])  indexCurrentPos = i;
    if(lastPos == statePhaseHall[i])  indexLastPos = i;
  }
  if(indexCurrentPos - indexLastPos == 1) return 1;
  else return 0;
}
uint8_t isFollowingPos(uint8_t currentPos, uint8_t lastPos){
// 100 -> 101 -> 001 -> 011 -> 010 -> 110
// tiến
  uint8_t indexCurrentPos = 0;
  uint8_t indexLastPos = 0;
  for(int i = 0; i < 6; i++){
    if(currentPos == statePhaseHall[i])  indexCurrentPos = i;
    if(lastPos == statePhaseHall[i])  indexLastPos = i;
  }
  if(indexLastPos - indexCurrentPos == 1) return 1;
  else return 0;
}
void setup() {
  Serial.begin(115200);
  TCCR2B = TCCR2B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz 9
  TCCR3B = TCCR3B & B11111000 | B00000001;   // for PWM frequency of 31372.55 Hz 5
  TCCR4B = TCCR4B & B11111000 | B00000001;   // for PWM frequency of 31372.55 Hz 6
  pinMode(HallU, INPUT_PULLUP);
  pinMode(HallV, INPUT_PULLUP);
  pinMode(HallW, INPUT_PULLUP);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  pinMode(EN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(HallU), isrU, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HallV), isrV, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HallW), isrW, CHANGE);
  digitalWrite(EN, HIGH);
  stateU = digitalRead(HallU);
  stateV = digitalRead(HallV);
  stateW = digitalRead(HallW);

}
void updateMotorControl(int step, uint8_t dir) {
    // Cập nhật trạng thái các chân điều khiển
    en;
    if(dir == 1){
      // chieu thuan
      if(phaseStates[step][0] == HIGH) analogWrite(pwmA, speed);
      else analogWrite(pwmA, 0);
      if(phaseStates[step][1] == HIGH) analogWrite(pwmB, speed);
      else analogWrite(pwmB, 0);
      if(phaseStates[step][2] == HIGH) analogWrite(pwmC, speed);
      else analogWrite(pwmC, 0);
    }
    else
    {
      if(phaseStates[step][0] == HIGH) analogWrite(pwmC, speed);
      else analogWrite(pwmC, 0);
      if(phaseStates[step][1] == HIGH) analogWrite(pwmB, speed);
      else analogWrite(pwmB, 0);
      if(phaseStates[step][2] == HIGH) analogWrite(pwmA, speed);
      else analogWrite(pwmA, 0);
    }
    

    // digitalWrite(pwmA, phaseStates[step][0]);
    // digitalWrite(pwmB, phaseStates[step][1]);
    // digitalWrite(pwmC, phaseStates[step][2]);
    // Cập nhật giá trị PWM cho chân EN để điều chỉnh tốc độ động cơ
    // analogWrite(en, 128); // Điều chỉnh giá trị PWM nếu cần
}
void loop() {
    // Điều khiển pha
    // updateMotorControl(stepCounter);

    // // Tăng biến đếm và reset nếu vượt quá số bước
    // stepCounter++;
    // printEncoder(updateHall());
    // if (stepCounter >= 6) {
    //     stepCounter = 0;
    // }
    // logHallinterrupt();


    if(Serial.available()){
      String data = Serial.readStringUntil('\n');
      speed = data.toInt();
      Serial.println(speed);
    }
    if(stateU == 1 && stateV == 0 && stateW == 0){
      
      // Serial.print("pha 0");
      updateMotorControl(1,0);
      // delay(10);
      
    }else if(stateU == 1 && stateV == 0 && stateW == 1){
      // Serial.print("pha 1");
      updateMotorControl(2,0);
      // delay(10);
    }else if(stateU == 0 && stateV == 0 && stateW == 1){
      // Serial.print("pha 2");
      updateMotorControl(3,0);
      // delay(10);
    }else if(stateU == 0 && stateV == 1 && stateW == 1){
      // Serial.print("pha 3");
      updateMotorControl(4,0);
      // delay(10);
    }else if(stateU == 0 && stateV == 1 && stateW == 0){
      // Serial.print("pha 4");
      updateMotorControl(5,0);
      // delay(10);
    }else if(stateU == 1 && stateV == 1 && stateW == 0){
      // Serial.print("pha 5");
      updateMotorControl(0,0);
      // delay(10);
    }else{
      //digitalWrite(EN, 0);
      offen;
    }
    

    

}