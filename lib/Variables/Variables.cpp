#include <Variables.h>

//qtr
const int setPoint = 7500;
const int SensorCount = 16;
int rawReadings[SensorCount];

//LED
const int red = 36;
const int blue = 37;
const int green = 38;

//colourSensor
const int S0 = 48;
const int S1 = 49;
const int S2 = 50;
const int S3 = 51;
const int sensorOut = 52;

//motorDriver
const int leftPins[] = {6, 22, 23};
const int rightPins[] = {7, 24, 25};

int stopDelay = 100;

//encoder
const int leftEncoderPins[] = {2,3};
const int rightEncoderPins[] = {18,19};

unsigned volatile long leftEncoder = 0;
unsigned volatile long rightEncoder = 0;
unsigned long encoderLeftCount = 0;
unsigned long encoderRightCount = 0;

//Speeds
int rightBase = 100;
int leftBase = 110;
int correctionMax = 40;
int maxSpeed = 180;

int turnRightBase = 160;
int turnLeftBase = 120;

//PID
int totalLineError = 0;
int prevLineError = 0;
int totalEncodeError = 0;
int prevEncoderError = 0;

const double eP = 0.4;
const double eI = 0;
const double eD = 2;

const double P = 0.05;
const double D = 0.1;
const double I = 0; //.001;



