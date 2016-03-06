#include <StackList.h>
#include <math.h>


/**
 * TO DO
 * 
 * -handle return to last save point method
 * --when returned, do not include turn made in the available turns 
 * 
 * -Turning
 * --when does the robot stop turning
 * 
 * hot & cold
 * -when the flame sensor picks up fire, attempt to find source
 * 
 */



//CONSTANTS
const int STRAIGHT = 2;  //Default when robot is not turning --> used for mapping
const int LEFT = 0;
const int RIGHT = 1;

//IRD
const int IRD_ANGLE = 45;
const int DISTANCE_BETWEEN_IRD_AND_USDS = 3;  //cm.

//Motor constants
const int MOTOR_SPEED_STEP_UP = 1;
const int MOTOR_SPEED_STEP_DOWN = 1;
const int MOTOR_TURN_SPEED_INCREASE = 5;
const int MOTOR_TURN_SPEED_DECREASE = 5;
const int STOP_MOVING = 0;

//Turning Deviation from wall, [cm]
const int TURN_POINT_THRESHOLD = 20;

//End of Corridor [cm]
const int END_OF_CORRIDOR = 20;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//COMPONENTS
//Buttons
const int btLeftPin;
const int btRightPin;
int leftBtState;
int rightBtState;

//Motors
const int rightMotorPin;
const int leftMotorPin;
//DEFAULT SPEED
int rightMotorSpeed = 10;
int leftMotorSpeed = 10;
//Distance traveled by motor
int rightMotorDist;
int leftMotorDist;

//SENSORS
/**
 * Value: Cm.
 */
//Ultra-sonic distance sensor
const int usdsLeftPin;
const int usdsFrontPin;
const int usdsRightPin;
int usdsLeft;
int usdsFront;
int usdsRight;

/**
 * 
 */
//IR-Distance Sensor
const int irDistLeftPin;
const int irDistRightPin;
int irDistLeft;
int irDistRight;

/**
 * Distance
 */
//IR-Proximity Sensor
const int irProxLeftPin;
const int irProxFrontPin;
const int irProxRightPin;
int irProxLeft;
int irProxFront;
int irProxRight;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Variables used for calculations and keeping track of distance/speed
 */

//Turning 
int turnChoice;
//Biases
int mainBias = NULL;
int leftBias = 50;
int rightBias = 50;

//Mapping
int distMoved;
StackList<int> turnStack; //keep track of all turns
StackList<int> distanceStack; //keep track of how far WallE moves after each turn

/**
 * Variables shown will be calculated based on IRDistance Sensor
 */
//Distance from corridors
//Previous
int prevDistLeft;
int prevDistRight;
//Curr
int currDistLeft;
int currDistRight;

//keep track of last chose path
//int lastPathChoice = NULL; 
boolean hasBeenReset = false;
boolean isRightPath = false;
boolean isLeftPaht = false;
boolean isForwPath = false;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  
}

void loop() {
  //Button 
  leftBtState = analogRead(btLeftPin);
  rightBtState = analogRead(btRightPin);

  //Motor 
  rightMotorDist = analogRead(rightMotorPin);
  leftMotorDist = analogRead(leftMotorPin);

  //USDS
  usdsLeft = convertMicrosecondsToCentimeters(pulseIn(usdsLeftPin,HIGH));
  usdsFront = convertMicrosecondsToCentimeters(pulseIn(usdsFrontPin,HIGH));
  usdsRight = convertMicrosecondsToCentimeters(pulseIn(usdsRightPin,HIGH));

  //IR-Distance   //Convert voltage reading to cm
  irDistLeft = analogRead(irDistLeftPin);
  irDistRight = analogRead(irDistRightPin);

  //IR-Proxy
  irProxLeft = analogRead(irProxLeftPin);
  irProxFront = analogRead(irProxFrontPin);
  irProxRight = analogRead(irProxRightPin);
  
  if(biasIsSet(leftBtState,rightBtState)){  //Once the bias has been set, wallE can begin moving through the obstacle course
    moveWallE();
  }
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
      mainBias = LEFT;
      break;
    case RIGHT:
      rightBias = 80;
      leftBias = 20;
      mainBias = RIGHT;
      break;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Manage the movement of WallE for scenarios:
 * -going along corridors
 * -managing turns
 * -confronting dead ends
 */
void moveWallE(){
  updateIRDistances();
  //Before meeting a turn/deadEnd/corridor, continue incrementing distance counter
  if(!isTurnAvailable() && !isDeadEnd()){
    keepWallEMovingParallelToCorridors();
  }else{
    stopWallE();
    if(isTurnAvailable()){
      setAvailablePaths();
      setTurnChoice();
      turnRobot(turnChoice);
    }else if(isDeadEnd()){
      hasBeenReset();
    }
  }
}

/**
 * Update distances for left/right/forward faces of robot using the IRD sensors
 */
void updateIRDistances(){
  updateIRDLeft();
  updateIRDRight();
}

void updateIRDLeft(){
  currDistLeft = (irDistLeft*cos(IRD_ANGLE)) - DISTANCE_BETWEEN_IRD_AND_USDS;
}
void updateIRDRight(){
  currDistRight = (irDistRight*cos(IRD_ANGLE)) - DISTANCE_BETWEEN_IRD_AND_USDS;
}

/**
 * Keep WallE moving parallel to the corridors, using the IR-Dist and USDS sensor readings.
 * 
 * 1]Cofirm distance away from wall
 * 2]In case of any increasing/decreasing values, speed/slow down right/left motor 
 * 
 * irDistLeft
 * irDistRight
 * 
 */
void keepWallEMovingParallelToCorridors(){
  //At beginning, move robot forward a given amount to start mapping out corridor
  if(prevDistLeft==0 && prevDistRight==0){
    prevDistLeft = currDistLeft;
    prevDistRight = currDistRight;
  }
  if(prevDistLeft!=currDistLeft || prevDistRight!=currDistRight){
    adjustSpeed();
  }
}

void stopWallE(){
  setSpeedOfMotors(STOP_MOVING,STOP_MOVING);
}

void adjustSpeed(){
  if(prevDistLeft > currDistLeft){
    leftMotorSpeed+=MOTOR_SPEED;
  }else if(prevDistRight > currDistRight){
    rightMotorSpeed+=MOTOR_SPEED;
  }
  setSpeedOfMotors(rightMotorSpeed,leftMotorSpeed);
}

/**
 * Set speed of motor
 */
void setSpeedOfMotors(rightSpeed, leftSpeed){
  rightMotorPin.analogWrite(rightSpeed);
  leftMotorPin.analogWrite(leftSpeed);
}

//////////////////////////////////////////////////////////////////////////////
//TURNING
/**
 * Check if any turning points are available.
 * Turning points are found when the current distance found initially by the IRD sensor is 
 * significantly greater than the right/left ping sensor
 */
boolean isTurnAvailable(){
  return (isRightTurn() || isLeftTurn());
}

/**
 * Check for existing right turn.
 */
boolean isRightTurn(){
  return(currDistRight-usdsRight > TURN_POINT_THRESHOLD);
}

/**
 * Check for existing left turn.
 */
boolean isLeftTurn(){
  return(currDistLeft-usdsLeft > TURN_POINT_THRESHOLD);
}

/**
 * Returns true if front corridor is less than constant END_OF_CORRIDOR
 */
boolean isFrontCorridorEnd(){
  return(usdsFront<=END_OF_CORRIDOR);
}

/**
 * If there is a dead end, return to the last save point
 */
boolean isDeadEnd(){
  return(!isRightTurn && !isLeftTurn && isFrontCorridorEnd);
}

//////////////////////////////////////////////////////////////////////////////
/**
 * Robot will make turns the main priority. In the case that turns are exhausted,
 * only then, robot will continue moving forward.
 */
 /**
  * Sets the available paths.
  */
void setAvailablePaths(){
  if(isRightTurn()){
    isRightPath = true;
  }
  if(isLeftTurn()){
    isLeftPath = true;
  }
  return true;
}

 /**
  * Check what paths the robot can take
  */
void setTurnChoice(){
  if(isRightPath && isLeftPath){
    handleBothAvailableTurns();
  }else if(isRightPath && !isLeftPath){ //only right turn available
    turnChoice = RIGHT;
  }else if(!isRightPath && isLeftPath){ //only left turn available
    turnRobot = LEFT;
  }
}

void handleBothAvailableTurns(){
  turnChoice = random(100);
  switch(mainBias){
    case LEFT:
      turnChoice = ((turnChoice > rightBias)?LEFT:RIGHT);
    break;
    case RIGHT:
      turnChoice = ((turnChoice > leftBias)?RIGHT:LEFT);
    break;
    case NULL:
      turnChoice = ((turnChoice >= 50)?LEFT:RIGHT);
    break;
  }
}

/**
 * Turn robot
 * 1]Save distance traveled to save point
 * 2]Turn robot
 * --Increase speed of wheel opposite to the direction being turned to
 * --Decrease speed of wheel in direction of turn
 * 3]Add save point
 */
void turnRobot(int turn){
  //Add the distance traveled to get to opening
  distanceStack.push(distMoved);
  switch(turn){
    case LEFT:
      rightSpeed+=MOTOR_TURN_SPEED_INCREASE;
      leftSpeed-=MOTOR_TURN_SPEED_DECREASE;
    break;
    case RIGHT:
      leftSpeed+=MOTOR_TURN_SPEED_INCREASE;
      rightSpeed-=MOTOR_TURN_SPEED_DECREASE;
    break;
  }
  addSavePoint(turn);
}

//////////////////////////////////////////////////////////////////////////////
//MAPPING
/**
 * Keep track of turns and distance ran by robot along the maze.
 */
void addSavePoint(int turn){
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
  //figure out if robot continues going forward in corrdior .....finish this

  switch(turnStack.pop()){
    case LEFT:
      isLeftPath = false;
    break;
    case RIGHT:
      isRightPath = false;
    break;
  }
  
  check if you can continue going straight on the original path, or you need to go through the left/right corridor
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * USDS: Convert microsec pings to cm
 */
long convertMicrosecondsToCentimeters(long microsec){
  return microsec/29/2;
}

/**
 * IR-Dist: Convert voltage readings to cm
 */
long convertVoltToCm(){
  
}


