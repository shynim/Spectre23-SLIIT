#include <Servo.h>

class Arm{
    public:
        Arm(int pin);

        void write(int angle);
        void grab();
    
        void armDown();
        void armUp();

    private:
        Servo servo;

};