#pragma once

#include "vex.h"

// Telemetry is kept separate from the raw controller output so the driver loop
// can display meaningful state without needing to know the controller internals.
struct HeadingTelemetry
{
  bool sensorReady = false;
  bool manualTurnActive = false;
  double currentHeadingDeg = 0.0;
  double targetHeadingDeg = 0.0;
  double headingErrorDeg = 0.0;
  double turnCommandPct = 0.0;
  double turnStickPct = 0.0;
  double integralTermPct = 0.0;
  double derivativeTermPct = 0.0;
};

struct HeadingUpdate
{
  double turnCommandPct = 0.0;
  HeadingTelemetry telemetry;
};

// This controller has two modes:
// 1. While the driver holds the right stick, the stick directly commands turn.
// 2. Once released, the controller captures the current heading and holds it.
class HeadingController
{
public:
  explicit HeadingController(vex::inertial &sensor);

  void calibrate(vex::controller &controller);
  void beginDriverControl();
  HeadingUpdate update(double rawTurnStickPct, bool resetPressed);

private:
  void clearDynamicState();

  vex::inertial &sensor_;
  double desiredHeadingDeg_ = 0.0;
  double previousTurnCommandPct_ = 0.0;
  double filteredTurnStickPct_ = 0.0;
  double headingIntegralDegSeconds_ = 0.0;
  double previousHeadingDeg_ = 0.0;
  double filteredHeadingRateDegPerSec_ = 0.0;
  bool sensorReady_ = false;
  bool wasManualTurnActive_ = false;
  bool wasResetPressed_ = false;
};
