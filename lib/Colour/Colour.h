#include <Arduino.h>
#include <Variables.h>
 
int redMin = 14; // Red minimum value
int redMax = 120; // Red maximum value
int greenMin = 15; // Green minimum value
int greenMax = 150; // Green maximum value
int blueMin = 13; // Blue minimum value
int blueMax = 120; // Blue maximum value
 
int redPW = 0;
int greenPW = 0;
int bluePW = 0;
 
int redValue;
int greenValue;
int blueValue;

int getRedPW() {

  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  int PW;
  PW = pulseIn(sensorOut, LOW);

  redValue = map(PW, redMin,redMax,255,0);
 
}

int getGreenPW() {
 
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  int PW;
  PW = pulseIn(sensorOut, LOW);
  
  greenValue = map(PW, greenMin,greenMax,255,0);
 
}

int getBluePW() {
 
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  int PW;
  PW = pulseIn(sensorOut, LOW);

  blueValue = map(PW, blueMin,blueMax,255,0);
 
}

char getColour(){
  
  getRedPW();
  getGreenPW();
  getBluePW();

  if(redValue > 150 && greenValue < 150 && blueValue < 150){
    return 'R';
  }else{
    return 'B';
  }
}