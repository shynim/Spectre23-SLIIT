#include <Arduino.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <Variables.h>
#include <MotorDriver.h>
#include <PID.h>

QTRSensors qtr;
MotorDriver driver;
PID pid;

uint16_t sensorValues[16];

void calibrate();
int getErr();

void printTuple(int a, int b)
{
  Serial.print(a);
  Serial.print("\t");
  Serial.println(b);
}

void calibrate(){
    
  digitalWrite(LED_BUILTIN, HIGH);

  /*leftForward(1);
  rightBackward(1);
  for (uint16_t i = 0; i < 50; i++)
  {
    // analogWrite(leftPWM, 100);
    // analogWrite(rightPWM, 100);
    qtr.calibrate();
  }

  leftBackward(1);
  rightForward(1);

  for (uint16_t i = 0; i < 55; i++)
  {
    // analogWrite(leftPWM, 100);
    // analogWrite(rightPWM, 100);
    qtr.calibrate();
  }
*/

    digitalWrite(LED_BUILTIN, LOW);
    driver.stop();

    qtr.calibrate();
    for (int i = 0; i < SensorCount; i++){
        qtr.calibrationOn.minimum[i] = 50;
        qtr.calibrationOn.maximum[i] = 2500;
    }
  
}

int getErr(){

    int position = qtr.readLineBlack(sensorValues);
    int err = (int)position - 7500;

    for (int i = 0; i < SensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print(" ");
        rawReadings[i] = sensorValues[i];
        sensorValues[i] = sensorValues[i] > 700 ? 1 : 0;
    }
    Serial.println();
    return err;
}

// void pid(int err)
// {

//   totalErr += err;

//   int correction = (int)(Kp * err + Ki * totalErr + Kd * (err-prevErr));
//   prevErr = err;

//   int leftSpeed = baseSpeed + correction;
//   int rightSpeed = baseSpeed - correction;

//   if (leftSpeed < 0)
//   {
//     leftSpeed = 0;
//   }

//   if (rightSpeed < 0)
//   {
//     rightSpeed = 0;
//   }

//   if (leftSpeed >= maxSpeed)
//   {
//     leftSpeed = maxSpeed;
//   }

//   if (rightSpeed >= maxSpeed)
//   {
//     rightSpeed = maxSpeed;
//   }

//    rightForward(rightSpeed);
//    leftForward(leftSpeed);
// }

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

    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){33, 34, 35, 36, 37, 38, 39, 40, 42, 43, 44, 45, 46, 47, 48, 49}, SensorCount);

    Serial.begin(9600);

    calibrate();

}

void loop(){

    int err = getErr();
    int correction = pid.getLineCorrection(err);

    driver.applyLinePid(correction * -1);

}