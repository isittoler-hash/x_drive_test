/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       IsitT                                                     */
/*    Created:      4/7/2026, 8:50:57 AM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller1;

// define your global instances of motors and other devices here
motor Pos_Y(PORT1, gearSetting::ratio18_1, true);
motor Pos_X(PORT3, gearSetting::ratio18_1, true);
motor Neg_Y(PORT7, gearSetting::ratio18_1, false);
motor Neg_X(PORT5, gearSetting::ratio18_1, false);
// Change PORT9 if your inertial sensor is plugged into a different port.
inertial HeadingSensor(PORT9);

namespace
{

  constexpr int kDriverLoopMs = 20;
  constexpr int kControllerStatusUpdateLoops = 5;
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDeadbandPct = 5.0;
  constexpr double kHeadingTurnStickDeadbandPct = 40.0;
  constexpr double kHeadingTurnStickFilterAlpha = 0.2;
  constexpr double kMaxManualTurnPct = 35.0;
  constexpr double kHeadingHoldKp = 0.28;
  constexpr double kHeadingHoldKi = 0.005;
  constexpr double kHeadingHoldKd = 0.05;
  constexpr double kHeadingIntegralZoneDeg = 10.0;
  constexpr double kHeadingHoldSettleErrorDeg = 1.5;
  constexpr double kMaxHeadingIntegralPct = 4.0;
  constexpr double kHeadingTurnSlewPctPerLoop = 3.0;
  constexpr double kHeadingDerivativeFilterAlpha = 0.15;
  constexpr double kMaxHeadingHoldPct = 18.0;

  // These aliases define the physical wheel order for the X-drive mix.
  // If translation or heading are mirrored on the robot, update this mapping
  // or the motor reversed flags instead of changing the control math.
  motor &FrontLeftMotor = Pos_Y;
  motor &FrontRightMotor = Neg_X;
  motor &BackLeftMotor = Pos_X;
  motor &BackRightMotor = Neg_Y;

  double desiredHeadingDeg = 0.0;
  double previousTurnCommandPct = 0.0;
  double filteredHeadingTurnStickPct = 0.0;
  double headingIntegralDegSeconds = 0.0;
  double previousHeadingDeg = 0.0;
  double filteredHeadingRateDegPerSec = 0.0;
  bool headingSensorReady = false;
  bool wasHeadingTurnInputActive = false;
  bool wasResetPressed = false;

  double clampValue(double value, double minValue, double maxValue)
  {
    if (value < minValue)
    {
      return minValue;
    }

    if (value > maxValue)
    {
      return maxValue;
    }

    return value;
  }

  double wrapDegrees180(double degrees)
  {
    while (degrees > 180.0)
    {
      degrees -= 360.0;
    }

    while (degrees <= -180.0)
    {
      degrees += 360.0;
    }

    return degrees;
  }

  double wrapDegrees360(double degrees)
  {
    while (degrees >= 360.0)
    {
      degrees -= 360.0;
    }

    while (degrees < 0.0)
    {
      degrees += 360.0;
    }

    return degrees;
  }

  double applyDeadband(double value, double deadbandPct)
  {
    return std::fabs(value) < deadbandPct ? 0.0 : value;
  }

  double applyScaledDeadband(double value, double deadbandPct)
  {
    const double magnitude = std::fabs(value);
    if (magnitude <= deadbandPct)
    {
      return 0.0;
    }

    const double scaledMagnitude =
        (magnitude - deadbandPct) * 100.0 / (100.0 - deadbandPct);
    return std::copysign(scaledMagnitude, value);
  }

  double moveToward(double currentValue, double targetValue, double maxStep)
  {
    if (targetValue > currentValue + maxStep)
    {
      return currentValue + maxStep;
    }

    if (targetValue < currentValue - maxStep)
    {
      return currentValue - maxStep;
    }

    return targetValue;
  }

  void spinDrive(double frontLeft, double frontRight, double backLeft,
                 double backRight)
  {
    const double maxMagnitude = std::fmax(
        std::fmax(std::fabs(frontLeft), std::fabs(frontRight)),
        std::fmax(std::fabs(backLeft), std::fabs(backRight)));

    if (maxMagnitude > 100.0)
    {
      const double scale = 100.0 / maxMagnitude;
      frontLeft *= scale;
      frontRight *= scale;
      backLeft *= scale;
      backRight *= scale;
    }

    FrontLeftMotor.spin(fwd, frontLeft, pct);
    FrontRightMotor.spin(fwd, frontRight, pct);
    BackLeftMotor.spin(fwd, backLeft, pct);
    BackRightMotor.spin(fwd, backRight, pct);
  }

  void stopDrive(brakeType mode = brakeType::brake)
  {
    FrontLeftMotor.stop(mode);
    FrontRightMotor.stop(mode);
    BackLeftMotor.stop(mode);
    BackRightMotor.stop(mode);
  }

  void printControllerStatus(double headingDeg, double targetHeadingDeg,
                             double headingErrorDeg, double turnCommandPct,
                             double turnStickPct, double integralTermPct,
                             double derivativeTermPct, bool turnInputActive,
                             bool sensorReady)
  {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    if (!sensorReady)
    {
      Controller1.Screen.print("No inertial");
      return;
    }

    Controller1.Screen.print("H%5.1f T%5.1f", headingDeg, targetHeadingDeg);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("E%5.1f U%5.1f", headingErrorDeg, turnCommandPct);
    Controller1.Screen.setCursor(3, 1);
    if (turnInputActive)
    {
      Controller1.Screen.print("RT%5.1f MAN ", turnStickPct);
    }
    else
    {
      Controller1.Screen.print("I%4.1f D%4.1f", integralTermPct,
                               derivativeTermPct);
    }
  }

  void calibrateHeadingSensor()
  {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    if (!HeadingSensor.installed())
    {
      Controller1.Screen.print("No inertial P9");
      headingSensorReady = false;
      return;
    }

    Controller1.Screen.print("Calibrating...");

    HeadingSensor.calibrate();
    while (HeadingSensor.isCalibrating())
    {
      wait(50, msec);
    }

    HeadingSensor.setHeading(0.0, degrees);
    desiredHeadingDeg = 0.0;
    headingSensorReady = true;

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Inertial ready");
  }

} // namespace
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  calibrateHeadingSensor();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void)
{
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void)
{
  if (headingSensorReady)
  {
    desiredHeadingDeg = HeadingSensor.heading(degrees);
  }
  previousTurnCommandPct = 0.0;
  filteredHeadingTurnStickPct = 0.0;
  headingIntegralDegSeconds = 0.0;
  previousHeadingDeg = desiredHeadingDeg;
  filteredHeadingRateDegPerSec = 0.0;
  wasHeadingTurnInputActive = false;
  wasResetPressed = false;
  int statusLoopCounter = 0;

  // User control code here, inside the loop
  while (1)
  {
    constexpr double kLoopSeconds = static_cast<double>(kDriverLoopMs) / 1000.0;
    const double rawFieldForward = Controller1.Axis3.position(pct);
    const double rawFieldRight = Controller1.Axis4.position(pct);
    const double rawHeadingTurn = Controller1.Axis1.position(pct);
    const double fieldForward =
        applyDeadband(rawFieldForward, kDeadbandPct);
    const double fieldRight =
        applyDeadband(rawFieldRight, kDeadbandPct);
    const double headingTurnStickTargetPct =
        applyScaledDeadband(rawHeadingTurn, kHeadingTurnStickDeadbandPct);
    filteredHeadingTurnStickPct +=
        (headingTurnStickTargetPct - filteredHeadingTurnStickPct) *
        kHeadingTurnStickFilterAlpha;
    const bool headingTurnInputActive =
        headingSensorReady && std::fabs(filteredHeadingTurnStickPct) >= 0.5;
    const double headingDeg =
        headingSensorReady ? HeadingSensor.heading(degrees) : 0.0;

    const bool resetPressed = Controller1.ButtonA.pressing();
    if (headingSensorReady && resetPressed && !wasResetPressed)
    {
      HeadingSensor.setHeading(0.0, degrees);
      desiredHeadingDeg = 0.0;
      previousTurnCommandPct = 0.0;
      filteredHeadingTurnStickPct = 0.0;
      headingIntegralDegSeconds = 0.0;
      previousHeadingDeg = 0.0;
      filteredHeadingRateDegPerSec = 0.0;
      wasHeadingTurnInputActive = false;
    }
    wasResetPressed = resetPressed;

    double turnCommand = 0.0;
    double headingErrorDeg = 0.0;
    if (headingSensorReady)
    {
      const double rawHeadingRateDegPerSec =
          wrapDegrees180(headingDeg - previousHeadingDeg) / kLoopSeconds;
      filteredHeadingRateDegPerSec +=
          (rawHeadingRateDegPerSec - filteredHeadingRateDegPerSec) *
          kHeadingDerivativeFilterAlpha;
      previousHeadingDeg = headingDeg;

      if (headingTurnInputActive)
      {
        desiredHeadingDeg = headingDeg;
        headingIntegralDegSeconds = 0.0;
        turnCommand =
            filteredHeadingTurnStickPct * kMaxManualTurnPct / 100.0;
      }
      else
      {
        if (wasHeadingTurnInputActive)
        {
          desiredHeadingDeg = headingDeg;
          headingIntegralDegSeconds = 0.0;
          filteredHeadingRateDegPerSec = 0.0;
          previousTurnCommandPct = 0.0;
        }

        headingErrorDeg = wrapDegrees180(desiredHeadingDeg - headingDeg);
        if (std::fabs(headingErrorDeg) <= kHeadingIntegralZoneDeg)
        {
          headingIntegralDegSeconds += headingErrorDeg * kLoopSeconds;
          const double maxIntegralDegSeconds =
              kMaxHeadingIntegralPct / kHeadingHoldKi;
          headingIntegralDegSeconds =
              clampValue(headingIntegralDegSeconds,
                         -maxIntegralDegSeconds,
                         maxIntegralDegSeconds);
        }
        else
        {
          headingIntegralDegSeconds = 0.0;
        }

        double requestedTurnCommand =
            headingErrorDeg * kHeadingHoldKp +
            headingIntegralDegSeconds * kHeadingHoldKi -
            filteredHeadingRateDegPerSec * kHeadingHoldKd;

        if (std::fabs(headingErrorDeg) <= kHeadingHoldSettleErrorDeg)
        {
          requestedTurnCommand = 0.0;
          headingIntegralDegSeconds = 0.0;
        }

        requestedTurnCommand = clampValue(requestedTurnCommand,
                                          -kMaxHeadingHoldPct,
                                          kMaxHeadingHoldPct);
        turnCommand =
            moveToward(previousTurnCommandPct, requestedTurnCommand,
                       kHeadingTurnSlewPctPerLoop);
      }

      if (headingTurnInputActive)
      {
        headingErrorDeg = 0.0;
      }
    }
    else
    {
      turnCommand = applyDeadband(rawHeadingTurn, kDeadbandPct);
    }
    previousTurnCommandPct = turnCommand;
    wasHeadingTurnInputActive = headingTurnInputActive;

    const double headingRad = headingDeg * kPi / 180.0;
    const double robotForward = headingSensorReady
                                    ? fieldForward * std::cos(headingRad) +
                                          fieldRight * std::sin(headingRad)
                                    : fieldForward;
    const double robotRight = headingSensorReady
                                  ? -fieldForward * std::sin(headingRad) +
                                        fieldRight * std::cos(headingRad)
                                  : fieldRight;

    const double frontLeftCommand = robotForward + robotRight + turnCommand;
    const double frontRightCommand = robotForward - robotRight - turnCommand;
    const double backLeftCommand = robotForward - robotRight + turnCommand;
    const double backRightCommand = robotForward + robotRight - turnCommand;

    if (fieldForward == 0.0 && fieldRight == 0.0 && turnCommand == 0.0)
    {
      stopDrive(brakeType::brake);
    }
    else
    {
      spinDrive(frontLeftCommand, frontRightCommand, backLeftCommand,
                backRightCommand);
    }

    statusLoopCounter += 1;
    if (statusLoopCounter >= kControllerStatusUpdateLoops)
    {
      statusLoopCounter = 0;
      printControllerStatus(
          headingDeg, desiredHeadingDeg, headingErrorDeg, turnCommand,
          filteredHeadingTurnStickPct,
          headingIntegralDegSeconds * kHeadingHoldKi,
          -filteredHeadingRateDegPerSec * kHeadingHoldKd,
          headingTurnInputActive, headingSensorReady);
    }

    wait(kDriverLoopMs, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
