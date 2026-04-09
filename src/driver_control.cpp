#include "driver_control.h"

#include "heading_controller.h"
#include "math_utils.h"
#include "robot_config.h"
#include "x_drive.h"

namespace
{

constexpr int kDriverLoopMs = 20;
constexpr int kControllerStatusUpdateLoops = 5;
constexpr double kTranslationDeadbandPct = 5.0;

HeadingController &headingController()
{
  static HeadingController controller(HeadingSensor);
  return controller;
}

void printControllerStatus(const HeadingTelemetry &telemetry)
{
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  if (!telemetry.sensorReady)
  {
    Controller1.Screen.print("No inertial");
    return;
  }

  Controller1.Screen.print("H%5.1f T%5.1f", telemetry.currentHeadingDeg,
                           telemetry.targetHeadingDeg);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("E%5.1f U%5.1f", telemetry.headingErrorDeg,
                           telemetry.turnCommandPct);
  Controller1.Screen.setCursor(3, 1);
  if (telemetry.manualTurnActive)
  {
    Controller1.Screen.print("RT%5.1f MAN ", telemetry.turnStickPct);
  }
  else
  {
    Controller1.Screen.print("I%4.1f D%4.1f", telemetry.integralTermPct,
                             telemetry.derivativeTermPct);
  }
}

} // namespace

void initializeDriverControl()
{
  headingController().calibrate(Controller1);
}

void runDriverControl()
{
  headingController().beginDriverControl();
  int statusLoopCounter = 0;

  while (true)
  {
    // Axis3/Axes4 keep handling translation while Axis1 is dedicated to
    // heading control. This keeps the driver loop itself simple.
    const double fieldForwardPct =
        math_utils::applyDeadband(Controller1.Axis3.position(vex::pct),
                                  kTranslationDeadbandPct);
    const double fieldRightPct =
        math_utils::applyDeadband(Controller1.Axis4.position(vex::pct),
                                  kTranslationDeadbandPct);
    const HeadingUpdate headingUpdate =
        headingController().update(Controller1.Axis1.position(vex::pct),
                                   Controller1.ButtonA.pressing());

    if (fieldForwardPct == 0.0 && fieldRightPct == 0.0 &&
        headingUpdate.turnCommandPct == 0.0)
    {
      x_drive::stop(vex::brakeType::brake);
    }
    else
    {
      x_drive::driveFieldRelative(fieldForwardPct, fieldRightPct,
                                  headingUpdate.telemetry.currentHeadingDeg,
                                  headingUpdate.telemetry.sensorReady,
                                  headingUpdate.turnCommandPct);
    }

    statusLoopCounter += 1;
    if (statusLoopCounter >= kControllerStatusUpdateLoops)
    {
      statusLoopCounter = 0;
      printControllerStatus(headingUpdate.telemetry);
    }

    vex::wait(kDriverLoopMs, vex::msec);
  }
}
