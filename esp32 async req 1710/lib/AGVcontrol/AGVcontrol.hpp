#ifndef _AGVCONTROL_H_
#define _AGVCONTROL_H_
#endif
#include <AccelStepper.h>
// #include <Arduino.h>
#include <MultiStepper.h>
// #include <math.h>
// #include <iostream>
// using namespace std;

#ifndef DIRL
#define DIRL 13
#endif
#ifndef STPL
#define STPL 14
#endif
#ifndef SLPL
#define SLPL 27
#endif
#ifndef RSTL
#define RSTL 26
#endif
#ifndef DIRR
#define DIRR 19
#endif
#ifndef STPR
#define STPR 33
#endif
#ifndef SLPR
#define SLPR 32
#endif
#ifndef RSTR
#define RSTR 23
#endif
#ifndef STEPS
#define STEPS 200
#endif

enum side { LEFT = 0, RIGHT = 1, BOTH = 2 };
enum state { DISABLE = 0, ENABLE = 1 };
enum rotation { CW = 0, CCW = 1 };

extern MultiStepper AGV;
  // step, dir
extern AccelStepper stepLeft;
extern AccelStepper stepRight;

class Robot {
 private:
  // Array used to store positions to be followed (left, right).
  long int positions[2] = {0};

  // Diameter of the wheel (mm).
  float _diam = 75;

  // Distance of the left wheel to the ICR (mm).
  float _distL = 10;

  // Distance of the right wheel to the ICR (mm).
  float _distR = 10;

  // Speed of the stepper motor (steps/s).
  float _speed = 100;

  // Circumference (length) of the wheel (mm).
  float _circ = _diam * PI;

  // Microsteps used by the driver (default: 4).
  int _microsteps = 4;

  // Full steps per revolution of the stepper motor (NEMA17: 200 steps).
  int _steps = STEPS;

 public:

  // Used to hold current position of right stepper.
  uint64_t _stpsR = 0;

  // Used to hold current position of left stepper.
  uint64_t _stpsL = 0;

  // Boolean state of the Robot instance
  bool running = false;
  bool moving = false;
  bool needsUpdate = false;
  bool hasBegun = false;
  bool hasFinished = false;

  /// @brief Sets wheel diameter to be used in calculations.
  /// @param diam The diameter of the wheel (mm)
  void setDiameter(float diam);

  /// @brief Sets max speed to be used with the steppers (or default to be used
  /// on straight distance). Note that stepper motors with smaller distances
  /// will travel at slower speeds. Uses internally configured microstep
  /// resolution to provide speed to equivalent full-step speed, therefore needs
  /// to be set after setMicrostepping()
  /// @param speed The speed in steps/s
  void setSpeed(float speed);

  //void setAccSpeed(float accspeed);
  //void setDeccSpeed(float decspeed);

  /// @brief Sets state (enabled or disabled) of the stepper motor.
  /// @param motside The side the stepper motor is located (LEFT, RIGHT or BOTH)
  /// @param State The state of the stepper motor (ENABLE or DISABLE)
  void setState(side motside, state State);

  /// @brief Sets microstepping to be used with the driver motor. Used internally to keep accurate measurements.
  /// @param micro The microstepping to be used (use the inverse value e.g. 1 for full step, 4 for one-quarter of a step, 8 for one-eighth)
  void setMicrostepping(int micro);

  /// @brief Sets internal step counters to zero. Used for position homing.
  void setHome();

  /// @brief Sets distance offset from wheel to the ICR (Instantaneous Center of Rotation).
  /// @param Side The side to be set (LEFT, RIGHT or BOTH)
  /// @param dist The distance in millimeters
  void setOffset(side Side, float dist);
  
  /// @brief Sets individual wheel speed (needs to be checked if it's redundant or not).
  /// @param Side The side to be set (LEFT, RIGHT or BOTH)
  /// @param speed The speed in steps/s
  void setWheelSpeed(side Side, int speed);

  /// @brief Returns the amount of steps needed to run given distance. Takes
  /// current microstepping resolution into account for calculations.
  /// @param distance The distance to be travelled in millimeters
  /// @param length The wheel circumference (recommended to use circ()) as input
  /// @return The amount of steps to be needed (truncated to integer values)
  long int calcSteps(float distance, float length);

  /// @brief Gets current distance in steps of actual stepper motor from its homing position.
  /// @param Side The side to be chosen (LEFT, RIGHT)
  /// @return The distance in steps 
  long int getPosition(side Side);

  /// @brief Returns the distance in steps to reach target position.
  /// @param Side The wheel chosen (LEFT, RIGHT)
  /// @return The distance in steps from current to target position
  long int distanceToGo(side Side);

  /// @brief Gets the diameter of the wheel set internally.
  /// @return The diameter of the wheel in millimeters
  float getDiam();

  /// @brief Gets the circumference (length) of the wheel set internally.
  /// @return The length of the wheel in millimeters
  float circ();
  
  /// @brief Prints in Serial the current positions of both stepper motors.
  void printPositions();

  /// @brief Begins the Robot instance, setting internally pins and adding steppers to the MultiStepper instance.
  /// @return If the initialization was successful (true)
  bool begin();

  /// @brief Gets current state of the steppers (if they are moving or not).
  /// @return Whether the steppers are travelling to target position or not
  bool getState();
  
  /// @brief Moves the Robot on an arc with given radius and degrees. This doesn't move the wheels by given degrees, but the robot as a whole.
  /// @param direction Whether the direction of movement is clockwise (CW) or counterclockwise (CCW)
  /// @param radius The radius in millimeters of the curve (set to 0 to make the robot spin on its own axis)
  /// @param degrees The angle of movement (recommended to stick with either positive values or same direction)
  void moveAngle(int direction, float radius, float degrees);

  /// @brief Moves the Robot a set amount of distance from its current position in a straight line. To set different distances to each wheel, look at curveDist().
  /// @param distance The distance in millimeters
  void moveDist(float distance);

  /// @brief Moves the Robot a set amount of steps from its current position in a straight line. To set different distances to each wheel, look at curveSteps().
  /// @param steps The distance to be traveled in millimeters
  void moveSteps(long int steps);

  /// @brief Moves the Robot to a given distance from its homing position. To set different distances to each wheel, look at curveToDist().
  /// @param distance The absolute distance from the homing position, in millimeters
  void moveToDist(float distance);

  /// @brief Moves the Robot's wheels individually by given distances from their current positions.
  /// @param distanceL The distance to be travelled by the left wheel, in millimeters
  /// @param distanceR The distance to be travelled by the right wheel, in millimeters
  void curveDist(float distanceL, float distanceR);

  /// @brief Moves the Robot's wheels individually by given steps from their current positions.
  /// @param stepL The distance to be travelled by the left wheel, in steps
  /// @param stepR The distance to be travelled by the right wheel, in steps
  void curveSteps(long int stepL, long int stepR);

  /// @brief Moves the Robot's wheels individually by given distances from their homing positions.
  /// @param distanceL The absolute distance of the left wheel from the homing position, in millimeters
  /// @param distanceR The absolute distance of the right wheel from the homing position, in millimeters
  void curveToDist(float distanceL, float distanceR);

  /// @brief Calls the steppers to move to their target positions. Needs to be called every time a new target position is set.
  void runSteppers();

  /// @brief Updates the internal variables to the respective positions of both stepper motors.
  void updateDist();

};
