#include "heading_controller.h"

#include "math_utils.h"

#include <cmath>

namespace
{

constexpr double kLoopSeconds = 0.02;
constexpr double kFallbackTurnDeadbandPct = 5.0;
constexpr double kHeadingTurnStickDeadbandPct = 40.0;
constexpr double kHeadingTurnStickFilterAlpha = 0.2;
constexpr double kManualTurnActivePct = 0.5;
constexpr double kMaxManualTurnPct = 35.0;

// The hold controller is intentionally conservative. The robot should settle
// into a heading instead of endlessly hunting a tiny error.
constexpr double kHeadingHoldAcceptErrorDeg = 3.0;
constexpr double kHeadingHoldKp = 0.28;
constexpr double kHeadingHoldKi = 0.005;
constexpr double kHeadingHoldKd = 0.05;
constexpr double kHeadingIntegralZoneDeg = 10.0;
constexpr double kMaxHeadingIntegralPct = 4.0;
constexpr double kHeadingTurnSlewPctPerLoop = 3.0;
constexpr double kHeadingDerivativeFilterAlpha = 0.15;
constexpr double kMaxHeadingHoldPct = 18.0;

} // namespace

HeadingController::HeadingController(vex::inertial &sensor) : sensor_(sensor)
{
}

void HeadingController::clearDynamicState()
{
  previousTurnCommandPct_ = 0.0;
  filteredTurnStickPct_ = 0.0;
  headingIntegralDegSeconds_ = 0.0;
  previousHeadingDeg_ = sensorReady_ ? sensor_.heading(vex::degrees) : 0.0;
  filteredHeadingRateDegPerSec_ = 0.0;
  wasManualTurnActive_ = false;
  wasResetPressed_ = false;
}

void HeadingController::calibrate(vex::controller &controller)
{
  controller.Screen.clearScreen();
  controller.Screen.setCursor(1, 1);
  if (!sensor_.installed())
  {
    controller.Screen.print("No inertial P9");
    sensorReady_ = false;
    desiredHeadingDeg_ = 0.0;
    clearDynamicState();
    return;
  }

  controller.Screen.print("Calibrating...");
  sensor_.calibrate();
  while (sensor_.isCalibrating())
  {
    vex::wait(50, vex::msec);
  }

  sensor_.setHeading(0.0, vex::degrees);
  sensorReady_ = true;
  desiredHeadingDeg_ = 0.0;
  clearDynamicState();

  controller.Screen.clearScreen();
  controller.Screen.setCursor(1, 1);
  controller.Screen.print("Inertial ready");
}

void HeadingController::beginDriverControl()
{
  if (sensorReady_)
  {
    desiredHeadingDeg_ = sensor_.heading(vex::degrees);
  }

  clearDynamicState();
}

HeadingUpdate HeadingController::update(double rawTurnStickPct, bool resetPressed)
{
  HeadingUpdate update;
  update.telemetry.sensorReady = sensorReady_;

  if (sensorReady_ && resetPressed && !wasResetPressed_)
  {
    sensor_.setHeading(0.0, vex::degrees);
    desiredHeadingDeg_ = 0.0;
    clearDynamicState();
  }
  wasResetPressed_ = resetPressed;

  const double filteredTurnTargetPct =
      math_utils::applyScaledDeadband(rawTurnStickPct,
                                      kHeadingTurnStickDeadbandPct);
  filteredTurnStickPct_ +=
      (filteredTurnTargetPct - filteredTurnStickPct_) *
      kHeadingTurnStickFilterAlpha;

  update.telemetry.turnStickPct = filteredTurnStickPct_;

  if (!sensorReady_)
  {
    update.turnCommandPct =
        math_utils::applyDeadband(rawTurnStickPct, kFallbackTurnDeadbandPct);
    update.telemetry.turnCommandPct = update.turnCommandPct;
    return update;
  }

  const bool manualTurnActive =
      std::fabs(filteredTurnStickPct_) >= kManualTurnActivePct;
  const double currentHeadingDeg = sensor_.heading(vex::degrees);
  const double rawHeadingRateDegPerSec =
      math_utils::wrapDegrees180(currentHeadingDeg - previousHeadingDeg_) /
      kLoopSeconds;
  filteredHeadingRateDegPerSec_ +=
      (rawHeadingRateDegPerSec - filteredHeadingRateDegPerSec_) *
      kHeadingDerivativeFilterAlpha;
  previousHeadingDeg_ = currentHeadingDeg;

  double headingErrorDeg = 0.0;
  double turnCommandPct = 0.0;

  if (manualTurnActive)
  {
    // While the driver is actively turning, do not fight them. Treat the live
    // heading as the next hold target so releasing the stick naturally "sticks"
    // to wherever the robot ended up.
    desiredHeadingDeg_ = currentHeadingDeg;
    headingIntegralDegSeconds_ = 0.0;
    turnCommandPct = filteredTurnStickPct_ * kMaxManualTurnPct / 100.0;
  }
  else
  {
    if (wasManualTurnActive_)
    {
      // Dropping directly from manual turn into hold can leave stale turn,
      // derivative, and integral state behind. Clearing that handoff prevents
      // the controller from continuing to rotate after the driver lets go.
      desiredHeadingDeg_ = currentHeadingDeg;
      headingIntegralDegSeconds_ = 0.0;
      filteredHeadingRateDegPerSec_ = 0.0;
      previousTurnCommandPct_ = 0.0;
    }

    headingErrorDeg =
        math_utils::wrapDegrees180(desiredHeadingDeg_ - currentHeadingDeg);

    if (std::fabs(headingErrorDeg) <= kHeadingHoldAcceptErrorDeg)
    {
      // Once the robot is "close enough", accept that real heading as good and
      // stop chasing tiny residual error that only causes oscillation.
      desiredHeadingDeg_ = currentHeadingDeg;
      headingErrorDeg = 0.0;
      headingIntegralDegSeconds_ = 0.0;
      filteredHeadingRateDegPerSec_ = 0.0;
      turnCommandPct = 0.0;
    }
    else
    {
      if (std::fabs(headingErrorDeg) <= kHeadingIntegralZoneDeg)
      {
        headingIntegralDegSeconds_ += headingErrorDeg * kLoopSeconds;
        const double maxIntegralDegSeconds =
            kMaxHeadingIntegralPct / kHeadingHoldKi;
        headingIntegralDegSeconds_ =
            math_utils::clampValue(headingIntegralDegSeconds_,
                                   -maxIntegralDegSeconds,
                                   maxIntegralDegSeconds);
      }
      else
      {
        headingIntegralDegSeconds_ = 0.0;
      }

      double requestedTurnPct = headingErrorDeg * kHeadingHoldKp +
                                headingIntegralDegSeconds_ * kHeadingHoldKi -
                                filteredHeadingRateDegPerSec_ * kHeadingHoldKd;
      requestedTurnPct =
          math_utils::clampValue(requestedTurnPct, -kMaxHeadingHoldPct,
                                 kMaxHeadingHoldPct);
      turnCommandPct =
          math_utils::moveToward(previousTurnCommandPct_, requestedTurnPct,
                                 kHeadingTurnSlewPctPerLoop);
    }
  }

  previousTurnCommandPct_ = turnCommandPct;
  wasManualTurnActive_ = manualTurnActive;

  update.turnCommandPct = turnCommandPct;
  update.telemetry.manualTurnActive = manualTurnActive;
  update.telemetry.currentHeadingDeg = currentHeadingDeg;
  update.telemetry.targetHeadingDeg = desiredHeadingDeg_;
  update.telemetry.headingErrorDeg = headingErrorDeg;
  update.telemetry.turnCommandPct = turnCommandPct;
  update.telemetry.integralTermPct =
      headingIntegralDegSeconds_ * kHeadingHoldKi;
  update.telemetry.derivativeTermPct =
      -filteredHeadingRateDegPerSec_ * kHeadingHoldKd;
  return update;
}
