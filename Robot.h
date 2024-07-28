#ifndef ROBOT
#define ROBOT
#include "Arm.h"
#include "FourD_Dict.h"
#include <BMI160Gen.h>

class Robot {
  public:
    Arm *arms;
    FourD_Dict q_values;
    const float alpha = 0.8;
    const float epsilon = 0.3;
    const float gamma = 0.8;
    const int select_pin = 10;
    float x_orientation, y_orientation, z_orientation;
    float correctionVector[3];

    Robot(int numberOfArms, Arm arms[], int timeDelay);
    ~Robot();
    int *getState();
    int **getPossibleMoves();
    int setState(int *state);
    void makeMove();
    void update_q_value(int *state, int *successor, float reward);
    float getReward();
    void calibrateBMI();


  private:
    int numArms;
    int *currentState;
    int **possibleMoves;
    int delayBetweenMoves;
    int numberOfPossibleMoves; //number of possible S'
    float lastReward;
    void populatePossibleMoves();
    void populate_q_values();
    int *chooseAction(bool greedy);
    const int BMICALIBRATIONREADINGS = 10000;
    const int i2c_addr = 0x68;
    const int BMI_READINGS = 100;
    const int ACCELEROMETERRANGE = 0X05;
    float *backgroundAcceleration;
    float *gyroOffsets;
    unsigned long prev_time;

};

#endif