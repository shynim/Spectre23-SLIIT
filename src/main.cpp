#include <Arduino.h>
#include <QTRSensors.h>
#include <Variables.h>
#include <MotorDriver.h>
#include <PID.h>
#include <SensorPanel.h>
#include <Arm.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Colour.h>

VL53L0X lox;
SensorPanel qtr(const_cast<uint8_t *>((const uint8_t[]) {33, 34, 35, 36, 37, 38, 39, 40, 42, 43, 44, 45, 46, 47, 48, 49}));
MotorDriver driver;
PID pid;
Arm arm;

void loxSetup(){

    delay(50);
    Wire.begin();

    lox.setTimeout(500);
    if (!lox.init())
    {
    while (1) {}
    }

    lox.startContinuous();
}

void buzz(){
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(200);
}

void lightRed(){
    digitalWrite(red,HIGH);
    digitalWrite(green,LOW);
    digitalWrite(blue,LOW);
}
void lightGreen(){
    digitalWrite(red,LOW);
    digitalWrite(green,HIGH);
    digitalWrite(blue,LOW);
}
void lightBlue(){
    digitalWrite(red,LOW);
    digitalWrite(green,LOW);
    digitalWrite(blue,HIGH);
}
void lightOff(){
    digitalWrite(red,LOW);
    digitalWrite(green,LOW);
    digitalWrite(blue,LOW);
}

void goStraight(){

    int errEncoder = leftEncoder - rightEncoder;
    int correction = pid.getEncoderCorrection(errEncoder);
    driver.applyEncoderPid(correction);

    //driver.forward(sonicLeftBase,sonicRightBase);
}

void calibrate(){
    
    digitalWrite(LED_BUILTIN, HIGH);

    driver.turnLeft(150,150);
    for (uint16_t i = 0; i < 50; i++){
    qtr.calibrate();
    }

    driver.turnRight(150,150);

    for (uint16_t i = 0; i < 50; i++){
    qtr.calibrate();
    }

    digitalWrite(LED_BUILTIN, LOW);
    driver.stop();

    while(lox.readRangeContinuousMillimeters() > 60){

    }

    delay(500);

  // qtr.calibrate();
  // for (int i = 0; i < SensorCount; i++){
  //     qtr.calibrationOn.minimum[i] = 50;
  //     qtr.calibrationOn.maximum[i] = 2500;
  // }
  
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
    driver.stop();

}

void pushBackward(int distance){
    encoderLeftCount = 0;
    encoderRightCount = 0;
    leftEncoder = 0;
    rightEncoder = 0;
    
    rightBase = rightBase * -1;
    leftBase = leftBase * -1;

    encoderRightCount = encoderRightCount + distance;
    encoderLeftCount = encoderLeftCount + distance;

    while(rightEncoder <= encoderRightCount || leftEncoder <= encoderLeftCount){
        goStraight();

    }
    driver.stop();
    rightBase = rightBase * -1;
    leftBase = leftBase * -1;

}

void turnRightTillMiddle(){

    qtr.read();
    while(qtr.panelReading[5] == 1){
        driver.turnRight(leftBase,rightBase);
        qtr.read();
    }
    
    qtr.read();
    while(qtr.panelReading[5] != 1){
        driver.turnRight(leftBase,rightBase);
        qtr.read();
    }
}

void turnLeftTillMiddle(){
    
    qtr.read();
    while(qtr.panelReading[10] == 1){
        driver.turnLeft(leftBase,rightBase);
        qtr.read();
    }

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
        turnRightBase = int(130+50/(1+pow(2.73,((50-dif)*0.05))));
        turnLeftBase = int(130+50/(1+pow(2.73,((50-dif)*0.05))));
        driver.turnRight(turnLeftBase, turnRightBase);

    }
    turnRightBase=180;
    turnLeftBase=180;
    encoderRightCount= encoderRightCount + 600;
    encoderLeftCount= encoderLeftCount + 600;
    while(rightEncoder <= encoderRightCount || leftEncoder <= encoderLeftCount)
    {
        driver.turnRight(turnLeftBase, turnRightBase);

    }
    encoderRightCount= encoderRightCount + 100;
    encoderLeftCount= encoderLeftCount + 100;
    while (rightEncoder <= encoderRightCount || leftEncoder <= encoderLeftCount)
    {
        int dif = leftEncoder - encoderLeftCount + 100;
        turnRightBase = int(180-50/(1+pow(2.73,((50-dif)*0.05))));
        turnLeftBase = int(180-50/(1+pow(2.73,((50-dif)*0.05))));
        driver.turnRight(turnLeftBase, turnRightBase);

    }
    driver.brake();

    turnLeftBase = 180;
    turnRightBase = 180;
    encoderLeftCount = 0;
    encoderRightCount = 0;
    leftEncoder = 0;
    rightEncoder = 0;

}

void straightenStart() {
    int reverseSpeed = 140;
    int leftSensor = 15, rightSensor = 0;
    qtr.read();
    const int limit = 100;
    while (qtr.panelReading[leftSensor] || qtr.panelReading[rightSensor]) {
        int count = 0;
        qtr.read();
        while (qtr.panelReading[leftSensor]) {
            driver.reverseLeft(reverseSpeed);
            qtr.read();
            if (count++ >= limit) {
                break;
            }
        }
        driver.stop();

        count = 0;
        qtr.read();
        while (qtr.panelReading[rightSensor]) {
            driver.reverseRight(reverseSpeed);
            qtr.read();
            if (count++ >= limit) {
                break;
            }
        }
        driver.stop();
    }
}

void countLeftOut1(){
    leftEncoder += 1;
}
void countRightOut1(){
    rightEncoder += 1;
}

void solveMaze(){
    while (true) {
        
        qtr.read();
        if (qtr.pattern == 1) {
            int correction = pid.getLineCorrection(qtr.error);
            driver.applyLinePid(correction * -1);

        }else{
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
            
            encoderRightCount = encoderRightCount + 100;
            encoderLeftCount = encoderLeftCount + 100;

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

            if(newPattern == 'T'){
                break;
            }

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

int grabCube(){

    int startDistance;
    if(lox.readRangeContinuousMillimeters() <= 250){
        startDistance = lox.readRangeContinuousMillimeters();
    }else{
        return;
    }

    delay(1000);

    arm.attachArm();
    arm.attachGripper();

    arm.spreadGripper();
    arm.armDown();

    delay(500);

    int pushDistance = (4.4 * startDistance);
    pushForward(pushDistance);

    delay(500);

    arm.grab();
    delay(500);
    arm.armUp();
    delay(1000);

    arm.detachGripper();
    arm.detachArm();

    delay(1000);

    return pushDistance;

}

char goThroughSquare(){

    int remainingDistance = 1400 - grabCube();
    int colourCount = 0;

    for(int i = 0; i < 100; i++){
        if(getColour() == 'B'){
            colourCount++;
        }else if(getColour() == 'R'){
            colourCount--;
        }
    }

    char colour;
    colour = colourCount > 0 ? 'B' : 'R';
    if(colour == 'B'){
        lightBlue();
    }else{
        lightRed();
    }

    pushForward(remainingDistance);
 
    return colour;

}


void goToEnd(char colour){
    while (true) {
        
        qtr.read();
        if (qtr.pattern == 1) {
            int correction = pid.getLineCorrection(qtr.error);
            driver.applyLinePid(correction * -1);

        }else if(qtr.isEnd){
            while(true){
                qtr.readWhite();

                if (qtr.pattern == 1) { //pid
                    int correction = pid.getLineCorrection(qtr.error);
                    driver.applyLinePid(correction * -1);
                }else{
                    driver.stop();
                    return;
                }

            }

        }else{
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
            
            encoderRightCount = encoderRightCount + 100;
            encoderLeftCount = encoderLeftCount + 100;

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
            
            qtr.read();
            if(qtr.isEnd){
            
                while(true){
                    qtr.readWhite();
                    if (qtr.pattern == 1) { //pid
                        int correction = pid.getLineCorrection(qtr.error);
                        driver.applyLinePid(correction * -1);
                    }else{ 
                        driver.stop();
                        return;
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

            if (t || (left && right) || (left && yRight) || (yLeft && right) || (yLeft && yRight)) {
                pattern = 'T';
            } else if (left || yLeft ) {
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
                    if (newPattern == 1) {
                        if(colour == 'R'){
                            turnLeftTillMiddle();
                        }else{

                        }
                    } else {
                        turnLeftTillMiddle();
                    }
                    break;

                case 'R':
                    if (newPattern == 1) {
                        if(colour == 'B'){
                            turnRightTillMiddle();
                        }else{

                        }
                    } else {
                        turnRightTillMiddle();
                    }
                    break;

                case 'T':
                    if(colour == 'R'){
                        turnLeftTillMiddle();
                    }else{
                        turnRightTillMiddle();
                    }
                    break;

                default:
                    turnBack();
                    break;
            }
            driver.stop();

        }
    }
}

void placeCube(){
    arm.attachArm();
    arm.attachGripper();

    arm.grab();
    delay(250);
    arm.armDown();
    delay(500);

    arm.spreadGripper();
    delay(250);

    pushBackward(200);

    arm.armUp();
    arm.grab();
    delay(1000);

    arm.detachGripper();
    arm.detachArm();

    delay(1000);


}

void returnToPickupSquare(char colour){
    turnBack();

    while (true) {
        
        qtr.read();
        if (qtr.pattern == 1) {
            int correction = pid.getLineCorrection(qtr.error);
            driver.applyLinePid(correction * -1);

        }else{
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
            
            encoderRightCount = encoderRightCount + 90;
            encoderLeftCount = encoderLeftCount + 90;

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

            if (t || (left && right) || (left && yRight) || (yLeft && right) || (yLeft && yRight)) {
                pattern = 'T';
            } else if (left || yLeft ) {
                pattern = 'L';
            } else if (right || yRight) {
                pattern = 'R';
            } else {
                pattern = 0;
            }
            driver.stop();
            
            qtr.read();
            char newPattern = qtr.pattern;

            if(newPattern == 'T'){
                straightenStart();
                pushForward(800);
                return;
            }

            switch (pattern) {
                case 'L':
                    if (newPattern == 1) {
                        if(colour == 'B'){
                            turnLeftTillMiddle();
                        }else{

                        }
                    } else {
                        turnLeftTillMiddle();
                    }
                    break;

                case 'R':
                    if (newPattern == 1) {
                        if(colour == 'R'){
                            turnRightTillMiddle();
                        }else{

                        }
                    } else {
                        turnRightTillMiddle();
                    }
                    break;

                case 'T':
                    if(colour == 'B'){
                        turnLeftTillMiddle();
                    }else{
                        turnRightTillMiddle();
                    }
                    break;

                default:
                    turnBack();
                    break;
            }
            driver.stop();

        }
    }



}

void botSetup(){

    loxSetup();

    arm.init(6, 7);
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

    pinMode(53,OUTPUT);

    calibrate();

    pushForward(80);

    arm.attachGripper();
    arm.attachArm();

    arm.writeArm(140);
    arm.writeGripper(110);

    arm.detachArm();
    arm.detachGripper();
}

void botLoop() {

    solveMaze();
    straightenStart();
    char colour = goThroughSquare();

    rightBase = 140;
    leftBase = 150;

    goToEnd(colour);
    placeCube();

    rightBase = 130;
    leftBase = 140;

    returnToPickupSquare(colour);

    driver.stop();
    delay(99999999);
    
}

void setup(){
    
    botSetup();
    
}

void loop(){

    // qtr.readWhite();
    // Serial.println(qtr.position);

    // int colourCount = 0;

    // for(int i = 0; i < 100; i++){
    //     if(getColour() == 'B'){
    //         colourCount++;
    //     }else if(getColour() == 'R'){
    //         colourCount--;
    //     }
    // }

    // char colour;
    // colour = colourCount > 0 ? 'B' : 'R';

    // if(colour == 'B'){
    //     lightBlue();
    // }else if(colour == 'R'){
    //     lightRed();
    // }


    // getRedPW();
    // getGreenPW();
    // getBluePW();

    // Serial.print(redValue);
    // Serial.print(" ");
    // Serial.print(blueValue);
    // Serial.print(" ");
    // Serial.println(greenValue);


    // digitalWrite(S2,LOW);
    // digitalWrite(S3,LOW);

    // int PW;
    // PW = pulseIn(sensorOut, LOW);
    // Serial.print(PW);
    // Serial.print(" ");

    // digitalWrite(S2,HIGH);
    // digitalWrite(S3,HIGH);

    // PW = pulseIn(sensorOut, LOW);
    // Serial.print(PW);
    // Serial.print(" ");

    // digitalWrite(S2,LOW);
    // digitalWrite(S3,HIGH);

    // PW = pulseIn(sensorOut, LOW);
    // Serial.print(PW);
    // Serial.println();

 


    botLoop();
    //Serial.println(lox.readRangeContinuousMillimeters());

    // arm.attachArm();
    // arm.armDown();

    // delay(1000);

    // arm.attachGripper();
    // arm.spreadGripper();

    // delay(1000);

    // arm.grab();

    // delay(1000);

    // arm.armUp();
    // delay(1000);
    
}