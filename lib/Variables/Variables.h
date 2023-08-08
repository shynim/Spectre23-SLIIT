#ifndef ROBOFEST2023_VARIABLES_H
#define ROBOFEST2023_VARIABLES_H

#define buzzer 53

//qtr
extern const int setPoint;
extern const int SensorCount;
extern int rawReadings[];

//LED
extern const int red;
extern const int green;
extern const int blue;

//colourSensor
extern const int S0;
extern const int S1;
extern const int S2;
extern const int S3;
extern const int sensorOut;

//motorDriver
extern const int leftPins[];
extern const int rightPins[];

extern int stopDelay;

//encoder
extern const int leftEncoderPins[];
extern const int rightEncoderPins[];

extern unsigned volatile long leftEncoder;
extern unsigned volatile long rightEncoder;
extern unsigned long encoderLeftCount;
extern unsigned long encoderRightCount;

//Speeds
extern int rightBase;
extern int leftBase;
extern int correctionMax;
extern int maxSpeed;

extern int turnRightBase;
extern int turnLeftBase;

//PID  
extern int prevEncoderError;  
extern int totalEncoderError;      
extern int prevLineError;
extern int totalLineError;

extern const double eP;
extern const double eI;
extern const double eD;
   
extern const double P;    
extern const double I;
extern const double D;

#endif //ROBOFEST2023_VARIABLES_H
