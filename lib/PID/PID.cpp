#include <PID.h>
#include <Arduino.h>
#include <Variables.h>

int PID::getEncoderCorrection(int error){
    double p = error * eP;
    double d = (error - prevEncoderError) *eD;

    prevEncoderError = error;

    int correction = (int)(p + d);

    return correction;
}

int PID::getLineCorrection(int error){
    double p = error * P;
    double i = totalLineError * I;
    double d = (error - prevLineError) * D;

    prevLineError = error;
    totalLineError += error;

    int correction = (int)(p + i + d);

    return correction;
}


