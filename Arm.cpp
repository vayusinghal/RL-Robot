#include "Arm.h"
#include <Arduino.h>
#include <Servo.h>

Arm::Arm(Servo servo, int min, int max, int res){
  // constructor function, given a pin number to link a servo to
  this->servo = servo;
  SERVOMIN = min;
  SERVOMAX = max;
  RESOLUTION = res;

  possibleStates = malloc(sizeof(int) * RESOLUTION);
  for (int i=0; i<RESOLUTION; i++){
    possibleStates[i] = (int) (SERVOMIN + (SERVOMAX-SERVOMIN)/RESOLUTION*i);
  }
}

int Arm::getState(){
  // return the current state of the arm
  return currentState;
}

int *Arm::getPossibleMoves(){
  // return a pointer to an array of possible moves for the arms
  return possibleStates;
}

void Arm::setState(int state){
  servo.write(state);
  currentState = state;
}

int Arm::getResolution(){
  return RESOLUTION;
}