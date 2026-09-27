#include "pid.h"
#include "config.h"

// ---------------------------------------------------------------------------
//  PID internal state
// ---------------------------------------------------------------------------
static float prevAngle = 0.0f;
static float integral = 0.0f;
static float filteredDerivative = 0.0f;

// ---------------------------------------------------------------------------
//  pidReset()
// ---------------------------------------------------------------------------
void pidReset(float initialAngle) {
    prevAngle = initialAngle;
    integral = 0.0f;
    filteredDerivative = 0.0f;
}



// ---------------------------------------------------------------------------
// Fixed timestep from DMP rate — more deterministic than micros()
const float dt = 1.0f / (float)DMP_RATE_HZ;
float iMax = PID_OUTPUT_MAX / (PID_KI + 0.001f);  // prevent division by zero

//  pidCompute()
//  Computes PID output based on the difference between current and target angle.
//  Returns a signed float:
//    positive → motors should spin forward (to catch forward tilt)
//    negative → motors should spin backward
// ---------------------------------------------------------------------------
float pidCompute(float currentAngle, float targetAngle) {

    float error = currentAngle - targetAngle;

    // Proportional
    float pTerm = PID_KP * error;

    // Integral (with anti-windup clamping)
    integral += error * dt;
    if (integral > iMax)  integral = iMax;
    if (integral < -iMax) integral = -iMax;
    float iTerm = PID_KI * integral;

    // Derivative on measurement (not error) to prevent derivative kick.
    // d(error)/dt = d(angle - setpoint)/dt = d(angle)/dt  (setpoint is constant)
    // Negate because increasing angle = increasing error.
    float rawDerivative = -(currentAngle - prevAngle) / dt;
    prevAngle = currentAngle;

    // First-order IIR low-pass filter on derivative to reduce noise
    filteredDerivative = PID_D_FILTER_ALPHA * filteredDerivative + (1.0f - PID_D_FILTER_ALPHA) * rawDerivative;

    float dTerm = PID_KD * filteredDerivative;

    // Sum and clamp
    float output = pTerm + iTerm + dTerm;
    if (output > PID_OUTPUT_MAX)  output = PID_OUTPUT_MAX;
    if (output < -PID_OUTPUT_MAX) output = -PID_OUTPUT_MAX;

    return output;
}
