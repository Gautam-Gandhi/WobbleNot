#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

/// Initialize MPU6050 DMP and attach INT0 interrupt on D2.
/// Blocks until DMP is ready. Prints status to Serial.
/// Returns true on success, false on failure.
bool imuInit();

/// Read ~CALIBRATION_SAMPLES DMP packets and average the pitch angle.
/// Call this once in setup() while the robot is held perfectly still and upright.
/// The averaged angle is stored internally as the balance reference point.
void imuCalibrate();

/// Check if new DMP data is available (interrupt-driven flag).
/// Returns true if a new angle was successfully read.
/// On success, writes the pitch angle (degrees) to *angle.
bool imuUpdate(float *angle);

/// Get the calibrated balance angle (degrees).
float imuGetBalanceAngle();

/// Extern flag set by the DMP interrupt ISR.
/// Main loop checks this to know when new data is available.
extern volatile bool dmpDataReady;

#endif // IMU_H
