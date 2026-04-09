#pragma once

// Initialize the devices needed for driver control, including inertial
// calibration if the sensor is present.
void initializeDriverControl();

// Run the driver-control loop until the competition template disables it.
void runDriverControl();
