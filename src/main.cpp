#include <Arduino.h>
#include "config.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"

// ---------------------------------------------------------------------------
//  Debug counter (compiled out when DEBUG_SERIAL == 0)
// ---------------------------------------------------------------------------
#if DEBUG_SERIAL
static uint8_t debugCounter = 0;
#endif

// ---------------------------------------------------------------------------
//  setup()
// ---------------------------------------------------------------------------
void setup() {
#if DEBUG_SERIAL
    Serial.begin(115200);
    while (!Serial) { ; }  // wait for Serial on USB-native boards (harmless on Nano)
    Serial.println(F("=== Balance Bot Phase 1 ==="));
#endif

    // 1. Initialize IMU + DMP + interrupt
    if (!imuInit()) {
#if DEBUG_SERIAL
        Serial.println(F("IMU init FAILED. Halting."));
#endif
        while (true) { ; }  // halt — no point continuing without IMU
    }

    // 2. Calibrate balance angle (hold the robot still and upright!)
    imuCalibrate();

    // 3. Initialize motors + Timer1 step engine
    motorsInit();

    // 4. Read a fresh angle and reset PID state with it
    float initAngle;
    while (!imuUpdate(&initAngle)) {}  // wait for first valid reading
    pidReset(initAngle);

    // Initialize built-in LED state indication
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    // Blink LED 5 times to indicate ready state
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }

#if DEBUG_SERIAL
    Serial.println(F("Ready! Release the robot."));
#endif
}

// ---------------------------------------------------------------------------
//  loop()
// ---------------------------------------------------------------------------
void loop() {
    // Wait for DMP data-ready interrupt flag
    if (!dmpDataReady) {
        return;
    }
    dmpDataReady = false;

    // Read pitch angle from DMP FIFO
    float angle;
    if (!imuUpdate(&angle)) {
        return;  // FIFO wasn't ready yet (shouldn't happen after interrupt)
    }

    float balanceAngle = imuGetBalanceAngle();

    // Safety: if the robot has fallen over, kill the motors
    float tiltError = angle - balanceAngle;
    if (abs(tiltError) > FALLEN_ANGLE_DEG) {
        motorsDisable();
    #if DEBUG_SERIAL
        Serial.println(F("FALLEN! Motors disabled."));
    #endif
        // Wait until the robot is picked back up and held upright for RESTART_HOLD_TIME_MS
        // Blink LED rapidly to indicate fallen state
        // Blink EVEN FASTER when robot is held upright (within RESTART_ANGLE_DEG)
        uint32_t lastBlinkTime = 0;
        uint32_t uprightHoldStart = 0;
        bool wasUpright = false;

        // Loop until held upright long enough to restart
        while (true) {
            bool isUpright = (abs(tiltError) <= RESTART_ANGLE_DEG);
            uint32_t blinkInterval = isUpright ? 50 : 300;  // faster blink when upright

            // Blink LED
            uint32_t currentTime = millis();
            if (currentTime - lastBlinkTime >= blinkInterval) {
                lastBlinkTime = currentTime;
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            }

            // Track upright hold time
            if (isUpright) {
                if (!wasUpright) {
                    uprightHoldStart = currentTime;  // start timer
                }
                // Check if held upright long enough
                if (currentTime - uprightHoldStart >= RESTART_HOLD_TIME_MS) {
                    break;  // exit fallen loop, will re-enable motors below
                }
            } else {
                uprightHoldStart = 0;  // reset timer
            }
            wasUpright = isUpright;

            // Read angle again
            while (!imuUpdate(&angle)) {}
            tiltError = angle - balanceAngle;
        }

        // Robot has been held upright long enough - restart
        // Get a fresh angle reading (drain any stale FIFO data)
        while (!imuUpdate(&angle)) {}
        pidReset(angle);  // reset PID with current angle to prevent derivative spike
        motorsEnable();
        digitalWrite(LED_BUILTIN, LOW);  // turn off LED
    #if DEBUG_SERIAL
        Serial.println(F("RESTART: Motors enabled, PID reset."));
    #endif
    }

    // Compute PID output
    float pidOutput = pidCompute(angle, balanceAngle);

    // Convert PID output to motor speed (both motors get the same speed for balancing)
    int16_t speed = (int16_t)pidOutput;
    setMotorSpeed(speed, speed);

    // Debug output at reduced rate (~4 Hz)
#if DEBUG_SERIAL
    if (++debugCounter >= DEBUG_PRINT_INTERVAL) {
        debugCounter = 0;
        Serial.print(F("A:"));
        Serial.print(angle, 2);
        Serial.print(F(" B:"));
        Serial.print(balanceAngle, 2);
        Serial.print(F(" E:"));
        Serial.print(tiltError, 2);
        Serial.print(F(" PID:"));
        Serial.println(pidOutput, 1);
    }
#endif
}
