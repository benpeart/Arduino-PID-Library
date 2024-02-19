#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION 1.2.1

class PID
{

public:
// Constants used in some of the functions below
#define AUTOMATIC 1
#define MANUAL 0
#define DIRECT 0
#define REVERSE 1
#define P_ON_M 0
#define P_ON_E 1

  // commonly used functions **************************************************************************
  PID(float Input, float Output, float Setpoint,                       // * constructor.  initializes the PID to the Input, Output, and
      float Kp, float Ki, float Kd, int POn, int ControllerDirection); //   Setpoint.  Initial tuning parameters are also set here.
                                                                       //   (overload for specifying proportional mode)

  PID(float Input, float Output, float Setpoint,              // * constructor.  initializes the PID to the Input, Output, and
      float Kp, float Ki, float Kd, int ControllerDirection); //   Setpoint.  Initial tuning parameters are also set here

  void SetMode(int Mode); // * sets PID to either Manual (0) or Auto (non-0)

  bool Compute(); // * performs the PID calculation.  it should be
                  //   called every time loop() cycles. ON/OFF and
                  //   calculation frequency can be set using SetMode
                  //   SetSampleTime respectively

  void SetOutputLimits(float Min, float Max); // * clamps the output to a specific range. 0-255 by default, but
                                              //   it's likely the user will want to change this depending on
                                              //   the application

  // available but not commonly used functions ********************************************************
  void SetTunings(float kp, float ki, // * While most users will set the tunings once in the
                  float kd);          //   constructor, this function gives the user the option
                                      //   of changing tunings during runtime for Adaptive control
  void SetTunings(float kp, float ki, // * overload for specifying proportional mode
                  float kd, int POn);

  void SetControllerDirection(int Direction); // * Sets the Direction, or "Action" of the controller. DIRECT
                                              //   means the output will increase when error is positive. REVERSE
                                              //   means the opposite.  it's very unlikely that this will be needed
                                              //   once it is set in the constructor.
  void SetSampleTime(int NewSampleTime);      // * sets the frequency, in Milliseconds, with which
                                              //   the PID calculation is performed.  default is 100

  float Input; // The Input, Output, and Setpoint variables
  float Output;
  float Setpoint;

  // Display functions ****************************************************************
  float GetKp();      // These functions query the pid for interal values.
  float GetKi();      //  they were created mainly for the pid front-end,
  float GetKd();      // where it's important to know what is actually
  int GetMode();      //  inside the PID.
  int GetDirection(); //

private:
  void Initialize();

  float dispKp; // * we'll hold on to the tuning parameters in user-entered
  float dispKi; //   format for display purposes
  float dispKd; //

  float kp; // * (P)roportional Tuning Parameter
  float ki; // * (I)ntegral Tuning Parameter
  float kd; // * (D)erivative Tuning Parameter

  int controllerDirection;
  int pOn;

  unsigned long lastTime;
  float outputSum, lastInput;

  unsigned long SampleTime;
  float outMin, outMax;
  bool inAuto, pOnE;
};
#endif
