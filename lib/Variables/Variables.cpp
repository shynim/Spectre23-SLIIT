#include <Variables.h>

//qtr
const int setPoint = 8000;
const int SensorCount = 16;

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
const int leftPins[] = {4, 22, 23};
const int rightPins[] = {5, 24, 25};

int stopDelay = 0;

//encoder
const int leftEncoderPins[] = {2,3};
const int rightEncoderPins[] = {18,19};

unsigned volatile long leftEncoder = 0;
unsigned volatile long rightEncoder = 0;
unsigned volatile long tempLeftEncoder = 0;
unsigned volatile long tempRightEncoder = 0;
unsigned long encoderLeftCount = 0;
unsigned long encoderRightCount = 0;

//Speeds
int rightBase = 130;
int leftBase = 140;
int correctionMax = 40;
int rightMaxSpeed = 200;
int leftMaxSpeed = 210;

int turnRightBase = 180;
int turnLeftBase = 180;

//PID
int totalLineError = 0;
int prevLineError = 0;
int totalEncodeError = 0;
int prevEncoderError = 0;

const double eP = 0.4;
const double eI = 0;
const double eD = 2;

double P = 0.03;
double D = 0.065;
double I = 0.00065; //.001;



