#include <AccelStepper.h>
#include <MultiStepper.h>

#include <AGVcontrol.hpp>
// #include <iostream>
// using namespace std;

MultiStepper AGV;
AccelStepper stepLeft(1, STPL, DIRL);
AccelStepper stepRight(1, STPR, DIRR);

void Robot::setDiameter(float diam) {
  _diam = diam;
  _circ = _diam * PI;
  Serial.println("diameter set: " + String(_diam));
  // cout << "diameter set: " << _diam << endl;
}

void Robot::setSpeed(float speed) {
  _speed = speed * _microsteps;
  stepLeft.setMaxSpeed(_speed);
  stepRight.setMaxSpeed(_speed);
  // cout << "speed set: " << _speed << endl;
  Serial.println("speed set: " + String(_speed));
}


/*
void Robot::setAccSpeed(float accspeed) {
  _accspeed = accspeed;
  // cout << "speed set: " << _accspeed << endl;
  Serial.println("acc speed set: " + String(_accspeed));
}

void Robot::setDeccSpeed(float decspeed) {
  _dccspeed = decspeed;
  // cout << "speed set: " << _dccspeed << endl;
  Serial.println("decc speed set: " + String(_dccspeed));
}
*/

void Robot::updateDist() {
  _stpsR = stepRight.currentPosition();
  _stpsL = stepLeft.currentPosition();
  
}

void Robot::setMicrostepping(int micro) {
  _microsteps = micro;
  Serial.println("microstepping set = " + String(_microsteps));
}

void Robot::setOffset(side Side, float offset) {
  if (Side == BOTH) {
    _distL = offset;
    _distR = offset;
    Serial.println("both offset set = " + String(_distL));
  } else if (Side == LEFT) {
    _distL = offset;
    Serial.println("left offset set = " + String(_distL));
  } else if (Side == RIGHT) {
    _distR = offset;
    Serial.println("right offset set = " + String(_distR));
  }
}

void Robot::setState(side motside, state State) {
  if (motside == LEFT) {
    digitalWrite(SLPL, (State == ENABLE ? HIGH : LOW));
    digitalWrite(RSTL, (State == ENABLE ? HIGH : LOW));
  }
  if (motside == RIGHT) {
    digitalWrite(SLPR, (State == ENABLE ? HIGH : LOW));
    digitalWrite(RSTR, (State == ENABLE ? HIGH : LOW));
  }
  if (motside == BOTH) {
    digitalWrite(SLPL, (State == ENABLE ? HIGH : LOW));
    digitalWrite(RSTL, (State == ENABLE ? HIGH : LOW));
    digitalWrite(SLPR, (State == ENABLE ? HIGH : LOW));
    digitalWrite(RSTR, (State == ENABLE ? HIGH : LOW));
  }
}

long int Robot::calcSteps(float distance, float length) {
  // cout << "circumference =" << _circ << endl;
  // Serial.println("circumference = " + String(_circ));
  float steps = (distance / length) * float(_steps * _microsteps);
  // cout << "distance = " << distance << "/tsteps = " << steps << endl;
  // Serial.println("distance (mm)= " + String(distance) + "\tsteps = " +
  // String(steps));
  return steps;
}

long int Robot::getPosition(side Side) {
  if (Side != BOTH) {
    return positions[Side];
  }
  return 0;
}

long int Robot::distanceToGo(side Side) {
  switch (Side) {
    case LEFT:
      return abs(stepLeft.distanceToGo());
      break;
    case RIGHT:
      return abs(stepRight.distanceToGo());
      break;
    case BOTH:
      return stepLeft.distanceToGo() > stepRight.distanceToGo()
                 ? abs(stepRight.distanceToGo())
                 : abs(stepLeft.distanceToGo());
      break;
    default:
      return -1;
      break;
  }
}

void Robot::printPositions() {
  Serial.println("Position of left motor:  " +
                 String(stepLeft.currentPosition()));
  Serial.println("Position of right motor: " +
                 String(stepRight.currentPosition()));
}

bool Robot::begin() {
  this->hasBegun = true;
  pinMode(SLPL, OUTPUT);
  pinMode(SLPR, OUTPUT);
  pinMode(RSTL, OUTPUT);
  pinMode(RSTR, OUTPUT);
  if (!AGV.addStepper(stepLeft)) return false;
  if (!AGV.addStepper(stepRight)) return false;
  setState(BOTH, ENABLE);
  setHome();
  return true;
}

bool Robot::getState() { return AGV.run(); }

void Robot::moveAngle(int direction, float radius, float degrees) {
  float leftPath = (direction == CCW ? ((radius - _distL) * 2 * M_PI)
                                     : ((radius + _distL) * 2 * M_PI));
  float rightPath = (direction == CW ? ((radius - _distR) * 2 * M_PI)
                                     : ((radius + _distR) * 2 * M_PI));
  leftPath = (leftPath * degrees) / 360;
  rightPath = (rightPath * degrees) / 360;
  curveDist(leftPath, rightPath);
}

void Robot::moveDist(float distance) {
  long int stps = calcSteps(distance, _circ);
  positions[LEFT] = stps + positions[LEFT];
  positions[RIGHT] = stps + positions[RIGHT];
  //Serial.println("Positions:\nLeft:  " + String(positions[LEFT]) + "\nRight: " + String(positions[RIGHT]));
  AGV.moveTo(positions);
  //Serial.println("move to done, running to position");
  this->moving = true;
  AGV.runSpeedToPosition();
  this->moving = false;
  //Serial.println("done");
}

void Robot::moveSteps(long int steps) {
  // cout << "moving " << steps << "steps" << endl;
 // Serial.println("moving " + String(steps) + " steps");
  positions[LEFT] = steps + positions[LEFT];
  positions[RIGHT] = steps + positions[RIGHT];
  AGV.moveTo(positions);
  this->moving = true;
  AGV.runSpeedToPosition();
  this->moving = false;
}

void Robot::moveToDist(float distance) {
  long int stps = calcSteps(distance, _circ);
  positions[LEFT] = stps;
  positions[RIGHT] = stps;
 // Serial.println("Positions:\nLeft:  " + String(positions[LEFT]) +
 //                "\nRight: " + String(positions[RIGHT]));
  AGV.moveTo(positions);
  this->moving = true;
  AGV.runSpeedToPosition();
  this->moving = false;
}

float Robot::getDiam(){
  return this->_diam;
}

void Robot::setWheelSpeed(side Side, int speed){
  if (Side == LEFT) {
    stepLeft.setMaxSpeed(speed);
    stepLeft.setSpeed(speed);
  }
  else if (Side == RIGHT) {
    stepRight.setMaxSpeed(speed);
    stepRight.setSpeed(speed);
  }
  Serial.println("stpl = " + String(stepLeft.speed()) + "\t stpl max = " + String(stepLeft.maxSpeed()));
  Serial.println("stpr = " + String(stepRight.speed()) + "\t stpr max = " + String(stepRight.maxSpeed()));
}

void Robot::curveDist(float distanceL, float distanceR) {
  long int stpsL = calcSteps(distanceL, _circ);
  long int stpsR = calcSteps(distanceR, _circ);
  positions[LEFT] = stpsL + positions[LEFT];
  positions[RIGHT] = stpsR + positions[RIGHT];
  //Serial.println("Positions:\nLeft:  " + String(positions[LEFT]) +
  //               "\nRight: " + String(positions[RIGHT]));
  AGV.moveTo(positions);
 // Serial.println("move to done, running to position");
  this->moving = true;
  AGV.runSpeedToPosition();
  this->moving = false;
  //Serial.println("done");
}

void Robot::curveSteps(long int stepL, long int stepR) {
  positions[LEFT] = stepL + positions[LEFT];
  positions[RIGHT] = stepR + positions[RIGHT];
  AGV.moveTo(positions);
  this->moving = true;
  AGV.runSpeedToPosition();
  this->moving = false;
}

void Robot::curveToDist(float distanceL, float distanceR) {
  long int stpsL = calcSteps(distanceL, _circ);
  long int stpsR = calcSteps(distanceR, _circ);
  positions[LEFT] = stpsL;
  positions[RIGHT] = stpsR;
  //Serial.println("Positions:\nLeft:  " + String(positions[LEFT]) +
 //                "\nRight: " + String(positions[RIGHT]));
  AGV.moveTo(positions);
  this->moving = true;
  AGV.runSpeedToPosition();
  this->moving = false;
}

void Robot::setHome() {
  //Serial.println("Positions:\nLeft:  " + String(positions[LEFT]) +
  //               "\nRight: " + String(positions[RIGHT]));

  stepLeft.setCurrentPosition(0);
  stepRight.setCurrentPosition(0);
  positions[LEFT] = 0;
  positions[RIGHT] = 0;
  //Serial.println("Homing:\nLeft:  " + String(stepLeft.currentPosition()) +
  //               "\nRight: " + String(stepRight.currentPosition()));
  Serial.println("Robot homed.");
}

void Robot::runSteppers() { AGV.run(); }

float Robot::circ(){ return _circ; }