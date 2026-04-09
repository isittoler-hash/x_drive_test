#pragma once

#include "vex.h"

#include <cmath>

namespace math_utils
{

// Clamp a value into the inclusive [min, max] range.
inline double clampValue(double value, double minValue, double maxValue)
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

// Wrap an angle into (-180, 180].
inline double wrapDegrees180(double degrees)
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

// Apply a simple symmetric deadband.
inline double applyDeadband(double value, double deadbandPct)
{
  return std::fabs(value) < deadbandPct ? 0.0 : value;
}

// Apply a deadband and then rescale the remaining range back to +/-100.
inline double applyScaledDeadband(double value, double deadbandPct)
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

// Limit how fast a command can change between control-loop iterations.
inline double moveToward(double currentValue, double targetValue, double maxStep)
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

} // namespace math_utils
