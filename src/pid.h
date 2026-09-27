#ifndef PID_H
#define PID_H

#include <Arduino.h>

/// Initialize / reset the PID controller state.
/// Pass the current angle so the derivative term doesn't spike on the first call.
/// Call once at startup and whenever you need to clear accumulated errors.
void pidReset(float initialAngle);

/// Compute PID output given the current angle and the target (balance) angle.
/// Returns a signed float: positive = motors should spin forward,
/// negative = motors should spin backward.
float pidCompute(float currentAngle, float targetAngle);

#endif // PID_H
