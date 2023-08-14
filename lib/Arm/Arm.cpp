#include <Arm.h>
#include <Arduino.h>

void Arm::init(int aP, int gP){
    armPin = aP;
    gripperPin = gP;
}

void Arm::attachArm(){
    arm.attach(armPin);
}

void Arm::detachArm(){
    arm.detach();
}

void Arm::attachGripper(){
    gripper.attach(gripperPin);
}

void Arm::detachGripper(){
    gripper.detach();
}

void Arm::writeArm(int angle){
    arm.write(angle);
}

void Arm::writeGripper(int angle){
    gripper.write(angle);
}

void Arm::spreadGripper(){
    gripper.write(80);
}

void Arm::grab(){
    gripper.write(110);
}

void Arm::armUp(){
    arm.write(150);
}

void Arm::armDown(){
    arm.write(28);
}