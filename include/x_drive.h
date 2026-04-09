#pragma once

#include "vex.h"

namespace x_drive
{

// Command field-relative translation plus a separate turn term.
void driveFieldRelative(double fieldForwardPct, double fieldRightPct,
                        double headingDeg, bool headingValid,
                        double turnPct);

void stop(vex::brakeType mode = vex::brakeType::brake);

} // namespace x_drive
