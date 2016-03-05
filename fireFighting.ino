#include <StackList.h>

//Constants
const int STRAIGHT = NULL;  //Default when robot is not turning --> used for mapping
const int LEFT = 0;
const int RIGHT = 1;

//Turning Biases
int leftBias = 50;
int rightBias = 50;

//Mapping
int distMoved;
StackList<int> turnStack; //keep track of all turns
StackList<int> distanceStack; //keep track of how far WallE moves after each turn

//COMPONENTS
//Buttons
const int btLeftPin;
const int btRightPin;
int leftBtState;
int rightBtState;

//Motors
const int rightMotorPin;
const int leftMotorPin;
int rightMotorSpeed;
int leftMotorSpeed;
//Distance traveled by motor
int rightMotorDistance;
int leftMotorDistance;

//SENSORS
//Ultra-sonic distance sensor
const int usdsLeftPin;
const int usdsFrontPin;
const int usdsRightPin;
int usdsLeft;
int usdsFront;
int usdsRight;
//IR-Distance Sensor
const int irDLeftPin;
const int irDRightPin;
int irDLeft;
int irDRight;
//IR-Proximity Sensor
const int irPLeftPin;
const int irPFrontPin;
const int irPRightPin;
int irPLeft;
int irPFront;
int irPRight;


void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:

  1]Get input from user for setting turn bias
  leftBtState = digitalRead(btLeftPin);
  rightBtState = digitalRead(btRightPin);
  
  if(biasIsSet(leftBtState,rightBtState)){  //Once the bias has been set, wallE can begin moving through the obstacle course
    moveWallE();
  }
  
}

/**
 * Check if the bias is set. In the case both are pressed, then you have the def 
 */
boolean biasIsSet(int leftBt, int rightBt){
  if(leftBt==HIGH && rightBt==LOW){
    setTurnBias(LEFT);
    return true;
  }else if(leftBt==LOW && rightBt==HIGH){
    setTurnBias(RIGHT);
    return true;
  }else if(leftBt==HIGH && rightBt==HIGH){      //leave the left && right bias as default and begin moving WallE
    return true;
  }
  return false;
}

/**
 * The percent chance for the robot will turn will be set by the button inputs. W/o pressing either button will leave 
 * a default bias of 50/50 chance for either left or right
 * 
  */
void setTurnBias(int bias){
  switch(bias){
    case LEFT:
      leftBias = 80;
      rightBias = 20;
      break;
    case RIGHT:
      rightBias = 80;
      leftBias = 20;
      break;
  }
}

/**
 * Manage the movement of WallE for scenarios:
 * -going along corridors
 * -managing turns
 * -confronting dead ends
 */
void moveWallE(){
  //move WallE

  //Before meeting a turn/deadEnd/corridor, continue incrementing distance counter
  //PSEUDOCODE
  while(!turn || !deadEnd || !corridor){
    distMoved++;
  }
  
  //before making a turn, save distance traveled to save point
  //TEMPORARY CODE EXAMPLE
  saveDistToSavePoint(   distance    );
  distanceStack.push(distance);
  addSavePoint(  turn   );
}

/**
 * Keep track of turns and distance ran by robot along the maze.
 */
void addSavePoint(int turn){
  //Add the distance traveled to get to opening
  distanceStack.push(distMoved);
  switch(turn){
    case LEFT:
      turnStack.push(LEFT);
    break;
    case RIGHT:
      turnStack.push(RIGHT);
    break;
  }  
  //reset distance counter
  distMoved = 0;
}

/**
 * 1]Move back distance traveled, while keeping track of 
 */
void goBackToLastSavePoint(){
  FLIP robot 180 degrees
  int distanceToGo = distanceStack.pop();
  move this distance
  turnStack.pop();    //figure out if robot continues going forward in corrdior .....finish this
  check if you can continue going straight on the original path, or you need to go through the left/right corridor
}




