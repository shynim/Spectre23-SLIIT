#ifndef ROBOFEST2023_PID_H
#define ROBOFEST2023_PID_H

class PID{
    public: 
        int getEncoderCorrection(int err);

        int getLineCorrection(int err);
        
    private:
    
};

#endif //ROBOFEST2023_PID_H