#include <Arm.h>
#include <Arduino.h>

Arm::Arm(int pin){
    servo.attach(pin);
}

void Arm::write(int angle){
    servo.write(angle);
}

void Arm::grab(){

}

void Arm::armUp(){

}

void Arm::armDown(){


}