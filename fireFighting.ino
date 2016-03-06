#include <StackList.h>
#include <math.h>

//CONSTANTS
const int STRAIGHT = NULL;  //Default when robot is not turning --> used for mapping
const int LEFT = 0;
const int RIGHT = 1;

//IRD
const int IRD_ANGLE = 45;
const int DISTANCE_BETWEEN_IRD_AND_USDS = 3;  //cm.

//Motor constants
const int MOTOR_SPEED_STEP_UP = 1;
const int MOTOR_SPEED_STEP_DOWN = 1;

//Turning Deviation from wall, [cm]
const int TURN_POINT_DEVIATION = 20;

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

//Turning Biases
int leftBias = 50;
int rightBias = 50;

//Mapping
int distMoved;
StackList<int> turnStack; //keep track of all turns
StackList<int> distanceStack; //keep track of how far WallE moves after each turn

//Distance from corridors
//Previous
int prevDistLeft;
int prevDistRight;
//Curr
int currDistLeft;
int currDistRight;

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
  updateDistances();
  //Before meeting a turn/deadEnd/corridor, continue incrementing distance counter
  if(!isTurnAvailable() && !isDeadEnd()){
    keepWallEMovingParallelToCorridors();
  }else{
    
    //before making a turn, save distance traveled to save point
    //TEMPORARY CODE EXAMPLE
    saveDistToSavePoint(   distance    );
    distanceStack.push(distance);
    addSavePoint(  turn   ); 
  }
}

/**
 * Update distances for left/right/forward faces of robot using the IRD sensors
 */
void updateDistances(){
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
    setSpeedOfMotors(rightMotorSpeed,leftMotorSpeed);
  }
}

void adjustSpeed(){
  if(prevDistLeft > currDistLeft){
    leftMotorSpeed+=MOTOR_SPEED;
  }else if(prevDistRight > currDistRight){
    rightMotorSpeed+=MOTOR_SPEED;
  }
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
 * Turning points are found when the current distance found initially by the IRD sensor is significantly greater than the prevDistFromLeft/Right
 * To be sure is when the calculated currDistLeft/Right is significantly greater than the 
 */
boolean isTurnAvailable(){
  return (isRightTurnPoint() || isLeftTurnPoint());
}

boolean isRightTurnPoint(){
  return (currDistRight > (prevDistRight + TURN_POINT_DEVIATION));
}

boolean isLeftTurnPoint(){
  return (currDistLeft > (prevDistRight + TURN_POINT_DEVIATION));
}

/**
 * 1]if the USDS return is approx. equal to IRD sensors && << const distance, then there is a dead end
 */
boolean isDeadEnd(){
  if(!isRightTurnPoint() && !isLeftTurnPoint()&& 
    ()){
      return true;
    }
}

//////////////////////////////////////////////////////////////////////////////
//MAPPING
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


