#include "x_drive.h"

#include "robot_config.h"

#include <cmath>

namespace
{

constexpr double kPi = 3.14159265358979323846;

// These aliases define the wheel order for the X-drive mix. If the robot's
// translation or turn direction is mirrored, fix the port mapping or reversed
// flags in robot_config.cpp instead of changing the math here.
vex::motor &FrontLeftMotor = Pos_Y;
vex::motor &FrontRightMotor = Neg_X;
vex::motor &BackLeftMotor = Pos_X;
vex::motor &BackRightMotor = Neg_Y;

void spinMixed(double frontLeftPct, double frontRightPct, double backLeftPct,
               double backRightPct)
{
  const double maxMagnitude = std::fmax(
      std::fmax(std::fabs(frontLeftPct), std::fabs(frontRightPct)),
      std::fmax(std::fabs(backLeftPct), std::fabs(backRightPct)));

  if (maxMagnitude > 100.0)
  {
    const double scale = 100.0 / maxMagnitude;
    frontLeftPct *= scale;
    frontRightPct *= scale;
    backLeftPct *= scale;
    backRightPct *= scale;
  }

  FrontLeftMotor.spin(vex::fwd, frontLeftPct, vex::pct);
  FrontRightMotor.spin(vex::fwd, frontRightPct, vex::pct);
  BackLeftMotor.spin(vex::fwd, backLeftPct, vex::pct);
  BackRightMotor.spin(vex::fwd, backRightPct, vex::pct);
}

} // namespace

namespace x_drive
{

void driveFieldRelative(double fieldForwardPct, double fieldRightPct,
                        double headingDeg, bool headingValid, double turnPct)
{
  // Convert the field-relative translation request into robot-relative motion
  // before applying the standard X-drive wheel mix.
  const double headingRad = headingDeg * kPi / 180.0;
  const double robotForward =
      headingValid ? fieldForwardPct * std::cos(headingRad) +
                         fieldRightPct * std::sin(headingRad)
                   : fieldForwardPct;
  const double robotRight =
      headingValid ? -fieldForwardPct * std::sin(headingRad) +
                         fieldRightPct * std::cos(headingRad)
                   : fieldRightPct;

  const double frontLeftPct = robotForward + robotRight + turnPct;
  const double frontRightPct = robotForward - robotRight - turnPct;
  const double backLeftPct = robotForward - robotRight + turnPct;
  const double backRightPct = robotForward + robotRight - turnPct;

  spinMixed(frontLeftPct, frontRightPct, backLeftPct, backRightPct);
}

void stop(vex::brakeType mode)
{
  FrontLeftMotor.stop(mode);
  FrontRightMotor.stop(mode);
  BackLeftMotor.stop(mode);
  BackRightMotor.stop(mode);
}

} // namespace x_drive
