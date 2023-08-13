#include <Servo.h>

class Arm{
    public:
        void init(int aP, int gP);

        void attachArm();
        void detachArm();
        void attachGripper();
        void detachGripper();

        void writeGripper(int angle);
        void writeArm(int angle);

        void spreadGripper();
        void grab();
    
        void armDown();
        void armUp();

    private:
        Servo arm;
        Servo gripper;
        
        int armPin;
        int gripperPin;

};