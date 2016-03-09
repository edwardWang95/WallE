#include <StackList.h>
#include <math.h>


/**
 * TO DO
 * 
 * fix logic involving turn choices
 * 
 * open field
 * 
 * hot & cold
 * -when the flame sensor picks up fire, attempt to find source
 * 
 * return to beginning
 * 
 * avoid teddybear
 * 
 * floating walls
 * 
 * am i able to setup a virtual maze testing enviroment for this code?
 * 
 */



//CONSTANTS
//Directions
const int LEFT = 0;
const int RIGHT = 1;
const int FORWARD = 2;  //Default when robot is not turning --> used for mapping

//IRD
const int IRD_ANGLE = 45;
const int DISTANCE_BETWEEN_IRD_AND_USDS = 3;  //cm.
const int IRD_USDS_DISTANCE_DEVIATION = 5;

//Motor constants
const int MOTOR_SPEED_STEP_UP = 1;
const int MOTOR_SPEED_STEP_DOWN = 1;
  //These const, manage the speeds of the left/right motors when turning
const int MOTOR_TURN_SPEED_DOMINANT = 15;
const int MOTOR_TURN_SPEED_RECESSIVE = 10;
const int STOP_MOVING = 0;
  //Used when robot is making an about face from dead end
const int MOTOR_SPIN_BACKWARD = 10;
const int MOTOR_SPIN_FORWARD = 10;

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
 * Used to find front corridors as well as increase accuracy of turn/searching algorithms
 */
//Ultra-sonic distance sensor
const int usdsLeftPin;
const int usdsFrontPin;
const int usdsRightPin;
int usdsLeft;
int usdsFront;
int usdsRight;

/**
 * Main distance sensor when finding walls/turns
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

/**
 * Flame Sensor
 */
const int flameSensorPin;
int flameDist;

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
int lastChosenPath = NULL; 
boolean hasBeenResetToLastSave = false;
boolean isRightPath = false;
boolean isLeftPaht = false;
boolean isForwPath = false;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  
}

void loop() { 
  getSensorInput();
  if(biasIsSet(leftBtState,rightBtState)){  //Once the bias has been set, wallE can begin moving through the obstacle course
    moveWallE();
  }
  
}

void getSensorInput(){
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

  //Flame Sensor
  flameDist = analogRead(flameSensorPin);
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
  if(isFireDetected){
    implementHotColdSearch();
  }else{
    handleMovement();
  }
}

void handleMovement(){
  //Before meeting a turn/deadEnd/corridor, continue incrementing distance counter
  if(!isTurnAvailable() && !isDeadEnd()){
    keepWallEMovingParallelToCorridors();
  }else{
    stopWallE();
    //check if there is an openField
    if(isWallEInOpenField()){
      handleOpenField();
    }else if(isTurnAvailable()){  //check if there are turns in the corridor
      if(!hasBeenResetToLastSave){
        setAvailablePaths();
      }
      setTurnChoice();
      turnRobot(turnChoice);
    }else if(isDeadEnd()){  //check if there is a dead end
      goBackToLastSavePoint();
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * When a fire is detected, immediately stop the robot
 */
boolean isFireDetected(){
  if(flameDist>0){
    stopWallE();
    return true;
  }
  return false;
}

/**
 * Hot and Cold search method
 */
void implementHotColdSearch(){
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boolean isTeddyBearDetected(){
  
}

void implementAvoidTeddyBear(){
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Arena
boolean isWallEInOpenField(){   //fix naming scheme later
  return(isLeftSideOpenField()||isRightSideOpenField());
}

/**
 * Check if the left side of the robot an open field
 */
boolean isLeftSideOpenField(){
  return(currDistLeft>prevDistLeft && usdsLeft>prevDistLeft);
}

boolean isRightSideOpenField(){
  return(currDistRight>prevDistLeft && usdsRight>prevDistLeft);
}

void handleOpenField(){
  
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
  if(!isDeadEnd()){
    isForwPath = true;
  }
  hasBeenReset=false;
  return true;
}

/**
 * Update the available path after exhausting a turning point and calling the resetToLastSavePoint().
 */
void updateAvailablePaths(int exhaustedPath){
  switch(exhaustedPath){
    case LEFT:
      isLeftPath = false;
    break;
    case RIGHT:
      isRightPath = false;
    break;
    cast FORWARD:
      isForwPath = false;
    break;
  }
}

 /**
  * Check what paths the robot can take
  */
void setTurnChoice(){
  if(!hasBeenResetToLastSavePoint){
    if(isRightPath && isLeftPath){
    handleBothAvailableTurns();
    }else if(isRightPath && !isLeftPath){ //only right turn available
      turnChoice = RIGHT;
    }else if(!isRightPath && isLeftPath){ //only left turn available
      turnChoice = LEFT;
    }
  }else{
    handleResetToLastSave();
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
      rightSpeed=MOTOR_TURN_SPEED_DOMINANT;
      leftSpeed=MOTOR_TURN_SPEED_RECESSIVE;
    break;
    case RIGHT:
      leftSpeed=MOTOR_TURN_SPEED_DOMINANT;
      rightSpeed=MOTOR_TURN_SPEED_RECESSIVE;
    case FORWARD:
      break;
    break;
  }
  moveRobotUntilTurnOrFlipIsComplete(rightSpeed,leftSpeed);
  addSavePoint(turn);
}

void moveRobotUntilTurnOrFlipIsComplete(int rightSpeed, int leftSpeed){
  setSpeedOfMotors(rightSpeed,leftSpeed);
  while(!(currDistLeft-IRD_USDS_DISTANCE_DEVIATION<=usdsLeft<=currDistLeft+IRD_USDS_DISTANCE_DEVIATION) 
        && 
        !(currDistRight-IRD_USDS_DISTANCE_DEVIATION<=usdsRight<=currDistRight-IRD_USDS_DISTANCE_DEVIATION)){
    getSensorInput();
    updateIRDDIstances();
  }
  resetMotorSpeedBackToOriginal();
}

void resetMotorSpeedBackToOriginal(){
  setSpeedOfMotors(rightMotorSpeed,leftMotorSpeed);
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
    case FORWARD:
      turnStack.push(FORWARD);
    break;
  }   
  //reset distance counter
  distMoved = 0;
}

/**
 * If the reset flag has not been raised, flip robot and go back. Else,
 * just turn back a certain distance. The turning of the robot for the else case
 * is already handled by the handleResetToLastSave method.
 */
void goBackToLastSavePoint(){
  if(!hasBeenResetToLastSavePoint){
    flipRobot180();
    hasBeenResetToLastSave = true;   
  }
  int distanceToGo = distanceStack.pop();


  
  move this distance !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //figure out if robot continues going forward in corrdior .....finish this
  
  lastChosenPath = turnStack.pop();
  updateAvailablePaths(lastChosenPath);
}


void handleResetToLastSave(){
  updateAvailablePaths(lastChosenPath);
  switch(lastChosenPath){
    case RIGHT:
      if(isLeftPath){ //There exists a LEFT path
        turnChoice = STRAIGHT;
        addSavePoint(LEFT);
      }else if(isForwPath){ //There is no LEFT turn, only a continuation STRAIGHT
        turnChoice = RIGHT;
        addSavePoint(STRAIGHT);
      }else{  //No left or straight path available, and in possibility of a hook turn, 
              //robot must go back another save point
        //Turn back to the corridor, and go back another save point
        turnRobot(LEFT);
        goBackToLastSavePoint();
      }
    break;
    case LEFT:
      if(isRightPath){  //Exists a RIGHT path
        turnChoice = STRAIGHT;
        addSavePoint(RIGHT);
      }else if(isForwPath){ //There is no RIGHT turn, only a continuation STRAIGHT
        turnChoice = LEFT;
        addSavePoint(STRAIGHT);
      }else{
        turnRobot(RIGHT);
        goBackToLastSavePoint();
      }
    break:
    case STRAIGHT:  //Turn case is down a path where turn points have been exhausted 
      goBackToLastSavePoint();
      return; //Because the robot resets to last save point again, we don't want to change status of the hasBeenResetToLastSave bool
    break;
  }
  hasBeenResetToLastSave = false; //Reset the reset switch
}

/**
 * Spins the wheels of WallE in opposite directions while keeping track of, and comparing the
 * distances of the IRD and USDS
 */
void flipRobot180(){
  leftSpeed = MOTOR_SPIN_BACKWARD;
  rightSPEED = MOTOR_SPIN_FORWARD;
  moveRobotUntilTurnOrFlipIsComplete(rightSpeed,leftSpeed);
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

