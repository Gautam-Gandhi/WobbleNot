#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

/// Configure STEP/DIR/ENABLE pins (DDR registers), set up Timer1 in CTC mode,
/// and enable the COMPA interrupt for step pulse generation.
/// Call this once in setup() AFTER imuInit().
void motorsInit();

/// Set motor speed for both motors.
/// speed is a signed value: positive = forward (catch forward tilt),
/// negative = backward. The sign convention respects INVERT_LEFT/RIGHT_MOTOR.
/// Magnitude maps to step interval (higher magnitude = faster stepping).
/// speedL and speedR are independent to allow differential steering later.
void setMotorSpeed(int16_t speedL, int16_t speedR);

/// Enable A4988 drivers (ENABLE pin LOW).
void motorsEnable();

/// Disable A4988 drivers (ENABLE pin HIGH) — motors go free-spinning.
void motorsDisable();

#endif // MOTORS_H
