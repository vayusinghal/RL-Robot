#ifndef ARM
#define ARM

#include <Servo.h>

class Arm {
  public:
    Arm(Servo servo, int min, int max, int res);
    int getState();
    int *getPossibleMoves();
    void setState(int state);
    int getResolution();


  private:
    Servo servo;
    int currentState;
    int SERVOMIN;
    int SERVOMAX;
    int RESOLUTION; //number of possible states
    int *possibleStates;
}; 

#endif