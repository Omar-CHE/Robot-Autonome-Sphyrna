#include <L298N.h>
#include <QTRSensors.h>

#define EN_D 9
#define EN_G 6
#define IN1_D 8
#define IN2_D 10
#define IN1_G 4
#define IN2_G 5

#define BASE_SPEED 127
#define BASE_POSITION 2000
#define Kp 0.08 
#define Kd 5
int lastError;

#define MAX_END_LINE_DURATION 400
#define MIN_END_LINE_DURATION 200

#define RED_HERRING_TRESHOLD 400
int noLineDuration = 0;

#define MIN_TURN_DURATION 50

// Moteurs (L298N)
L298N motorDroit(EN_D,IN1_D,IN2_D); 
L298N motorGauche(EN_G,IN1_G,IN2_G); 

// Capteurs Avant (QTR 5 voies)
QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

// Capteurs Arrieres (TCRT5000)
#define CAPTEUR_ARRIERE_GAUCHE 12
#define CAPTEUR_ARRIERE_DROIT 11

unsigned long time;
unsigned long highSensorCountTime;

void setup() {
  Serial.begin(9600);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A4, A3, A2}, SensorCount);

  pinMode(CAPTEUR_ARRIERE_GAUCHE, INPUT);
  pinMode(CAPTEUR_ARRIERE_DROIT, INPUT);
  
  calibrateSensors(2);
  
  lastError = 0;
  time  = millis();
  highSensorCountTime = time;
}

bool uTurnRight = true;
void loop() {
  int position = qtr.readLineWhite(sensorValues);
  unsigned long deltaT = millis() - time;
  time  = millis();
  if(!isLineDetected()){
    if(noLineDuration == 0)
      uTurnRight = true;
    if(readRearLeft())
      uTurnRight = false;
    noLineDuration += deltaT;
    if(noLineDuration >= RED_HERRING_TRESHOLD){
      drive(-BASE_SPEED, -BASE_SPEED);
      delay(RED_HERRING_TRESHOLD);
      drive(-BASE_SPEED*3/4, BASE_SPEED*3/4);
      delay(MIN_TURN_DURATION);
      bool lineFound = false;
      for (int i = 0; i < 50; i++) // Scans for  a line to the right
      {
        delay(10);
        if(isLineDetected())
          break;
      }
      if (!isLineDetected())
      {
        drive(BASE_SPEED*3/4,-BASE_SPEED*3/4);
        delay(500 + MIN_TURN_DURATION); // We go back to our initial position
        while(!isLineDetected()){ // Scans for a line to the left
          delay(10);
        }
      }
      noLineDuration = 0;
      
    } else {
      drive(BASE_SPEED,BASE_SPEED);
    }
    
  } else {
    noLineDuration = 0;
    int error = position - BASE_POSITION;
    int correction = Kp*error + Kd*(error - lastError);
    correction = constrain(correction, -BASE_SPEED, BASE_SPEED);
    int leftSpeed = BASE_SPEED + correction;
    int rightSpeed = BASE_SPEED - correction;
    lastError = error;
    drive(leftSpeed, rightSpeed);
  }
  // Finish line
  int highSensorCount = 0;
  qtr.readCalibrated(sensorValues);
  for(int i = 0; i < SensorCount; i++) {
    if(sensorValues[i] < 500) highSensorCount++;  // Most sensors on line
  }
  if(highSensorCount >= 4){
    if(MIN_END_LINE_DURATION < time-highSensorCountTime && time-highSensorCountTime < MAX_END_LINE_DURATION){
      stop();
      while(1);
    } else if (time-highSensorCountTime > MAX_END_LINE_DURATION){
      highSensorCountTime = time;
    }
  }
  delay(5);
}

void drive(int left, int right) {

  motorDroit.setSpeed(constrain(abs(right),0,255));
  if (right > 10) { motorDroit.forward(); } 
  else if (right < -10) { motorDroit.backward(); }
  else motorDroit.stop();

  motorGauche.setSpeed(constrain(abs(left),0,255));
  if (left > 10) { motorGauche.forward(); } 
  else if (left < -10){ motorGauche.backward(); }
  else motorGauche.stop();
}

void stop(){
  motorDroit.stop();
  motorGauche.stop();
}

void calibrateSensors(int cycles) {
  int sweepSpeed = 250;  // Speed for the active motor
  drive(128,128);
  delay(10);
  stop();
  for (int i = 0; i < cycles; i++) {
    twitch(-sweepSpeed,sweepSpeed);
    stop();
    qtr.calibrate();
    delay(200);
    qtr.calibrate();
    twitch(sweepSpeed,-sweepSpeed);
    stop();
    qtr.calibrate();
    delay(200);
    qtr.calibrate();
    twitch(sweepSpeed,-sweepSpeed);
    stop();
    qtr.calibrate();
    delay(200);
    qtr.calibrate();
    twitch(-sweepSpeed,sweepSpeed);
    stop();
    qtr.calibrate();
  }
}

void twitch(int x,int y) {
  for (int j = -10; j <= 10; j++) {
    if(j==0) continue;
    drive(x/abs(j), y/abs(j));
    qtr.calibrate();
    delay(10);
  }
  stop();
}

bool isLineDetected(){
  bool lineDetected = false;
  qtr.readCalibrated(sensorValues);

  for(int i = 0; i < 5; i++){
    if(sensorValues[i] < 500)
      lineDetected = true;
  }
  return lineDetected;
}

void uTurn(bool right){
  int direction = right ? 1 : -1;
  drive(direction*BASE_SPEED/2, -1*direction*BASE_SPEED/2);
  delay(MIN_TURN_DURATION);
  while(!isLineDetected()){ // Keeps turning around until we find the line
    drive(direction*BASE_SPEED,-1*direction*BASE_SPEED);
    delay(5);
  }
}

bool readRearLeft(){ // Maybe change this to qtr.readCalibrated()
  return digitalRead(CAPTEUR_ARRIERE_GAUCHE) == HIGH;
}