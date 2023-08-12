#include <Arduino.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <Variables.h>
#include <MotorDriver.h>
#include <PID.h>
#include <SensorPanel.h>
#include <Arm.h>

SensorPanel qtr(const_cast<uint8_t *>((const uint8_t[]) {33, 34, 35, 36, 37, 38, 39, 40, 42, 43, 44, 45, 46, 47, 48, 49}));
MotorDriver driver;
PID pid;
Arm gripper(7);
Arm arm(6);

void calibrate();

void goStraight(){

    int errEncoder = leftEncoder - rightEncoder;
    int correction = pid.getEncoderCorrection(errEncoder);
    driver.applyEncoderPid(correction);

    //driver.forward(sonicLeftBase,sonicRightBase);
}

void calibrate(){
    
  digitalWrite(LED_BUILTIN, HIGH);

  driver.turnLeft(100,100);
  for (uint16_t i = 0; i < 50; i++){
    qtr.calibrate();
  }

  driver.turnRight(100,100);

  for (uint16_t i = 0; i < 50; i++){
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);
  driver.stop();


  // qtr.calibrate();
  // for (int i = 0; i < SensorCount; i++){
  //     qtr.calibrationOn.minimum[i] = 50;
  //     qtr.calibrationOn.maximum[i] = 2500;
  // }
  
}

void cellStart(){
    encoderLeftCount = 0;
    encoderRightCount = 0;
    leftEncoder = 0;
    rightEncoder = 0;

    encoderRightCount = encoderRightCount + 200;
    encoderLeftCount = encoderLeftCount + 200;

    while (rightEncoder <= encoderRightCount || leftEncoder <= encoderLeftCount){
        int dif = leftEncoder - encoderLeftCount + 200;
        rightBase = (rightBase - 20) + int(dif/(200/20));
        leftBase = (leftBase - 20) + int(dif/(200/20));
        goStraight();
    }

}

void cellBrake(){

    encoderLeftCount = 0;
    encoderRightCount = 0;
    leftEncoder = 0;
    rightEncoder = 0;
    
    encoderRightCount = encoderRightCount + 150;
    encoderLeftCount = encoderLeftCount + 150;

    while(rightEncoder <= encoderRightCount || leftEncoder <= encoderLeftCount){
        int dif = leftEncoder - encoderLeftCount + 150;
        rightBase = rightBase - int(dif/7.5);
        leftBase = leftBase - int(dif/7.5);
        goStraight();

    }
    leftBase = 95;
    rightBase = 95;
    driver.stop();
    
}

void pushForward(int distance){
    encoderLeftCount = 0;
    encoderRightCount = 0;
    leftEncoder = 0;
    rightEncoder = 0;
    
    encoderRightCount = encoderRightCount + distance;
    encoderLeftCount = encoderLeftCount + distance;

    while(rightEncoder <= encoderRightCount || leftEncoder <= encoderLeftCount){
        goStraight();

    }

}

void turnRightTillMiddle(){
    qtr.read();
    while(qtr.panelReading[5] != 1){
        driver.turnRight(leftBase,rightBase);
        qtr.read();
    }
}

void turnLeftTillMiddle(){
    qtr.read();
    while(qtr.panelReading[10] != 1){
        driver.turnLeft(leftBase,rightBase);
        qtr.read();
    }
}

void turnBack(){
    encoderLeftCount = 0;
    encoderRightCount = 0;
    leftEncoder = 0;
    rightEncoder = 0;
    encoderRightCount= encoderRightCount + 100;
    encoderLeftCount= encoderLeftCount + 100;
    while (rightEncoder <= encoderRightCount || leftEncoder <= encoderLeftCount)
    {
        int dif = leftEncoder - encoderLeftCount + 100;
        turnRightBase = int(80+50/(1+pow(2.73,((50-dif)*0.05))));
        turnLeftBase = int(80+50/(1+pow(2.73,((50-dif)*0.05))));
        driver.turnRight(turnLeftBase, turnRightBase);

    }
    turnRightBase=160;
    turnLeftBase=120;
    encoderRightCount= encoderRightCount + 500;
    encoderLeftCount= encoderLeftCount + 500;
    while(rightEncoder <= encoderRightCount || leftEncoder <= encoderLeftCount)
    {
        driver.turnRight(turnLeftBase, turnRightBase);

    }
    encoderRightCount= encoderRightCount + 100;
    encoderLeftCount= encoderLeftCount + 100;
    while (rightEncoder <= encoderRightCount || leftEncoder <= encoderLeftCount)
    {
        int dif = leftEncoder - encoderLeftCount + 100;
        turnRightBase = int(130-50/(1+pow(2.73,((50-dif)*0.05))));
        turnLeftBase = int(130-50/(1+pow(2.73,((50-dif)*0.05))));
        driver.turnRight(turnLeftBase, turnRightBase);

    }
    driver.brake();

    turnLeftBase = 120;
    turnRightBase = 160;
    encoderLeftCount = 0;
    encoderRightCount = 0;
    leftEncoder = 0;
    rightEncoder = 0;

}

void countLeftOut1(){
    leftEncoder += 1;
}
void countRightOut1(){
    rightEncoder += 1;
}

void setup(){

    driver.init(const_cast<int *>(leftPins), const_cast<int *>(rightPins));

    pinMode(leftEncoderPins[0], INPUT);
    pinMode(leftEncoderPins[1], INPUT);
    pinMode(rightEncoderPins[0], INPUT);
    pinMode(rightEncoderPins[1], INPUT);

    pinMode(buzzer,OUTPUT);

    attachInterrupt(digitalPinToInterrupt(leftEncoderPins[0]), countLeftOut1, RISING);
    attachInterrupt(digitalPinToInterrupt(leftEncoderPins[1]), countLeftOut1, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncoderPins[0]), countRightOut1, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncoderPins[1]), countRightOut1, RISING);

    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    
    pinMode(sensorOut, INPUT);
    
    digitalWrite(S0,HIGH);
    digitalWrite(S1,LOW);

    pinMode(red,OUTPUT);
    pinMode(green,OUTPUT);
    pinMode(blue,OUTPUT);

    pinMode(LED_BUILTIN,OUTPUT);

    Serial.begin(9600);

    calibrate();
    driver.stop();
    delay(3000);

    arm.write(140);
    gripper.write(110);

}

void BotLoop() {
    while (true) {
        qtr.read();

        if (qtr.pattern == 1) { //pid
            int correction = pid.getLineCorrection(qtr.error);
            driver.applyLinePid(correction * -1);
        } else {
            char pattern = qtr.pattern;
            bool left = pattern == 'L';
            bool right = pattern == 'R';
            bool t = pattern == 'T';

            bool yLeft = false;
            bool yRight = false;

            int pushDistance = 100 - (leftEncoder - tempLeftEncoder);

            encoderLeftCount = 0;
            encoderRightCount = 0;
            leftEncoder = 0;
            rightEncoder = 0;
            
            encoderRightCount = encoderRightCount + 150;
            encoderLeftCount = encoderLeftCount + 150;

            int tCount = 0;
            while(rightEncoder <= encoderRightCount || leftEncoder <= encoderLeftCount){
                
                goStraight();

                qtr.read();
                
                if (qtr.pattern == 'L') {
                    left = true; 
                } else if (qtr.pattern == 'R') {
                    right = true;
                } else if (qtr.pattern == 'T') {
                    t = true;
                    tCount++;
                }

                for(int i = 0; i <= 5; i++){
                    if(qtr.panelReading[i] == 1){
                        yRight = true;
                    }
                    if(qtr.panelReading[15 - i] == 1){
                        yLeft = true;
                    
                    }
                }
            }

            encoderLeftCount = 0;
            encoderRightCount = 0;
            leftEncoder = 0;
            rightEncoder = 0;
            
            encoderRightCount = encoderRightCount + pushDistance;
            encoderLeftCount = encoderLeftCount + pushDistance;

            while(rightEncoder <= encoderRightCount || leftEncoder <= encoderLeftCount){
                goStraight();
            }

            if (t || (left && right)) {
                pattern = 'T';
            } else if (left || yLeft) {
                pattern = 'L';
            } else if (right || yRight) {
                pattern = 'R';
            } else {
                pattern = 0;
            }
            driver.stop();
            
            qtr.read();
            char newPattern = qtr.pattern;

            switch (pattern) {
                case 'L':
                    turnLeftTillMiddle();
                    break;

                case 'R':
                    if (newPattern == 1) {
                        
                    } else {
                        turnRightTillMiddle();
                    }
                    break;

                case 'T':
                    turnLeftTillMiddle();
                    break;

                default:
                    turnBack();
                    break;
            }
            driver.stop();

        }
    }
}


void loop(){

//    qtr.read();
//    for(int i = 0; i < SensorCount; i++){
//     Serial.print(qtr.rawReadings[i]);
//     Serial.print(" ");
//    }
//    Serial.println();

//   int correction = pid.getLineCorrection(qtr.error);
//   driver.applyLinePid(correction * -1);

    //BotLoop();
    // turnBack();
    // delay(100000);
    //driver.forward(95,95);

    // delay(1000);
    // arm.write(120);
    // gripper.write(90);

    // delay(1000);
    // arm.write(50);
    // gripper.write(110);
    
    }