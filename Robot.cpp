#include <BMI160Gen.h>
#include <MatrixMath.h>

#include "Robot.h"
#include <Arduino.h>

#define MIN_INT (-2147483647)

inline int pow(int base, int exp){
  int result = 1;
  for (int i = 0; i<exp; i++){
    result *= base;
  }
  return result;
}

// must be called AFTER the Serial monitor is initialised
Robot::Robot(int numberOfArms, Arm *arms, int timeDelay){
  numArms = numberOfArms;
  this->arms = arms;
  currentState = malloc(sizeof(int) * numArms); // doesn't need to be freed
  delayBetweenMoves = timeDelay;

  populatePossibleMoves();
  q_values = FourD_Dict();
  populate_q_values();
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Initialising BMI160...");
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  backgroundAcceleration = malloc(sizeof(float) * 3);
  gyroOffsets = malloc(sizeof(float) * 3);
  backgroundAcceleration[0] = backgroundAcceleration[1] = backgroundAcceleration[2] = 
  gyroOffsets[0] = gyroOffsets[1] = gyroOffsets[2] = 0;
  calibrateBMI();
}

Robot::~Robot(){
  free(backgroundAcceleration);
  free(gyroOffsets);
}

int *Robot::getState(){
  // return the state of the robot, in the form [state_arm1, state_arm2, ...]
  for (int i=0; i<numArms; i++){
    currentState[i] = arms[i].getState();
  }
  return currentState;
}

int **Robot::getPossibleMoves(){
  return possibleMoves;
}

void Robot::populate_q_values(){
  // number of S->S' = (arm1.res * arm2.res * ... * armN.res)^2
  int numPossMoves = 1;
  for (int i = 0; i < numArms; i++){
    numPossMoves *= arms[i].getResolution();
  }
  numPossMoves *= numPossMoves;

  //THIS PART IS NOT GENERALISED FOR >2 ARMS
  //need int * of 4 integers
  int res0 = arms[0].getResolution();
  int res1 = arms[1].getResolution();
  int *moves0 = arms[0].getPossibleMoves();
  int *moves1 = arms[1].getPossibleMoves();

  for (int i=0; i<res0; i++){
    for (int j=0; j<res1; j++){
      for (int k=0; k<res0; k++){
        for (int l=0; l<res1; l++){
          int *key = malloc(sizeof(int)*4);
          key[0] = moves0[i];
          key[1] = moves1[j];
          key[2] = moves0[k];
          key[3] = moves1[l];
          q_values.putValue(key, 0.01);
        }
      }
    }
  }
}

void Robot::populatePossibleMoves(){
  numberOfPossibleMoves = 1;
  for (int i=0; i<numArms; i++){
    numberOfPossibleMoves *= arms[i].getResolution();
  }
  possibleMoves = malloc(sizeof(int *) * numberOfPossibleMoves); // doesn't need to be freed

  // code below not generalised
  int res0 = arms[0].getResolution();
  int res1 = arms[1].getResolution();
  int *moves0 = arms[0].getPossibleMoves();
  int *moves1 = arms[1].getPossibleMoves();
  int currentMoveIdx = 0;

  for (int i=0; i<res0; i++){
    for (int j=0; j<res1; j++){
      possibleMoves[currentMoveIdx] = malloc(sizeof(int)*2);
      possibleMoves[currentMoveIdx] = moves0[i];
      possibleMoves[currentMoveIdx++] = moves1[j];
    }
  }
}

void Robot::makeMove(){
  int *nextState = chooseAction(0);
  for (int i=0; i<numArms; i++){
    arms[i].setState(nextState[i]);
  }
  float reward = lastReward = getReward();
  update_q_value(getState(), nextState, reward);
  currentState = nextState;
}

int *Robot::chooseAction(bool greedy){
  float local_epsilon = epsilon;
  if (greedy == 1){
    local_epsilon = 0.0;
  }
  if (((float)rand() / RAND_MAX) < local_epsilon){
    // pick random successor state (generate random number between 0 and (arm1.resolution*arm2.resolution)
    // use that index from the first case of (arm1.currentState, arm2.currentState, -, -)
    int desiredIdx = rand() % (arms[0].getResolution() * arms[1].getResolution());
    return possibleMoves[desiredIdx];
  }
  else{
    // pick best q_value successor state
    int *partial = malloc(sizeof(int) * 2); // freed
    partial[0] = currentState[0]; partial[1] = currentState[1];

    FourD_Dict::Node *first = q_values.getFirst2DKey(partial);
    free(partial);
    FourD_Dict::Node *last = q_values.getLast2DKey(first);
    float maxQ = MIN_INT;
    FourD_Dict::Node *maxNode = first;

    while (first != last->next){
      if (first->data > maxQ){
        maxNode = first;
        maxQ = first->data;
      }
      first = first->next;
    }

    return (maxNode->key)+2;
  }
}

void Robot::update_q_value(int *state, int *successor, float reward){
  // using current q-values for future choices (Q-learning)
  int *key = malloc(sizeof(int)*4); // freed
  for (int i=0; i<numArms; i++){
    key[i] = state[i];
    key[i+numArms] = successor[i+numArms];
  }
  FourD_Dict::Node *current_node = q_values.getNode(key); // node containing the value Q 

  // now need to get value Q'
  int *future_key = malloc(sizeof(int) * 4); // freed
  for (int i=0; i<numArms; i++){
    future_key[i] = future_key[i+numArms];
  }
  state = successor;
  currentState = state; // update the robot's state, so we can choose next action
  successor = chooseAction(1); // greedily get next action for q value calculation

  for (int i=0; i<numArms; i++){
    future_key[i+numArms] = successor[i];
  }

  float current_Q = current_node->data;
  float future_Q = q_values.getValue(future_key); // this now contains value of Q'

  float new_Q = current_Q + alpha * lastReward + gamma*future_Q - current_Q;
  q_values.putValue(key, new_Q);

  free(key);
  free(future_key);
}

//assuming we are using x-axis acceleration:
float Robot::getReward(){
  int16_t ax, ay, az;         // raw accelerometor values
  float result = 0;
  for (int i=0; i<BMI_READINGS; i++){
    BMI160.readAccelerometer(ax, ay, az);
    result += (float)ax / pow(2, 15) * (float)ACCELEROMETERRANGE / BMI_READINGS;
  }

  return result;

}

void Robot::calibrateBMI(){
  int gx, gy, gz, ax, ay, az;
  for (int i=0; i<BMICALIBRATIONREADINGS; i++){
    BMI160.readGyro(gx, gy, gz);
    BMI160.readAccelerometer(ax, ay, az);
    gyroOffsets[0] += gx/(float)BMICALIBRATIONREADINGS;
    gyroOffsets[1] += gy/(float)BMICALIBRATIONREADINGS;
    gyroOffsets[2] += gz/(float)BMICALIBRATIONREADINGS;
    backgroundAcceleration[0] += ax/(float)BMICALIBRATIONREADINGS;
    backgroundAcceleration[1] += ay/(float)BMICALIBRATIONREADINGS;
    backgroundAcceleration[2] += az/(float)BMICALIBRATIONREADINGS;
  }
}