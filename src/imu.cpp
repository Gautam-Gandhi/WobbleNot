#include "imu.h"
#include "config.h"
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

// ---------------------------------------------------------------------------
//  Module state
// ---------------------------------------------------------------------------
static MPU6050 mpu;

// DMP packet buffer
static uint8_t  fifoBuffer[64];
static uint16_t packetSize;

// Calibrated balance angle (degrees)
static float balanceAngle = 0.0f;

// Interrupt flag — set by ISR, cleared by imuUpdate()
volatile bool dmpDataReady = false;

// ---------------------------------------------------------------------------
//  DMP data-ready ISR (attached to INT0 / D2)
// ---------------------------------------------------------------------------
static void dmpISR() {
    dmpDataReady = true;
}

// ---------------------------------------------------------------------------
//  imuInit()
// ---------------------------------------------------------------------------
bool imuInit() {
    Wire.begin();
    Wire.setClock(400000);  // 400 kHz fast-mode I2C

    mpu.initialize();

    // Verify connection
    if (!mpu.testConnection()) {
#if DEBUG_SERIAL
        Serial.println(F("MPU6050 connection failed!"));
#endif
        return false;
    }
#if DEBUG_SERIAL
    Serial.println(F("MPU6050 connected."));
#endif

    // Initialize DMP
    uint8_t devStatus = mpu.dmpInitialize();

    // Supply your own gyro/accel offsets here if you have them.
    // These are reasonable defaults; the auto-calibration in setup() handles
    // the balance-angle offset, not the raw sensor offsets.
    // mpu.setXGyroOffset(220);
    // mpu.setYGyroOffset(76);
    // mpu.setZGyroOffset(-85);
    // mpu.setZAccelOffset(1788);

    if (devStatus != 0) {
#if DEBUG_SERIAL
        Serial.print(F("DMP init failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
#endif
        return false;
    }

    // Calibrate gyro and accelerometer
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
#if DEBUG_SERIAL
    Serial.println(F("Sensor offsets calibrated."));
    mpu.PrintActiveOffsets();
#endif

    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();

    // Attach hardware interrupt on D2 (INT0), RISING edge
    pinMode(MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpISR, RISING);

#if DEBUG_SERIAL
    Serial.println(F("DMP ready. Interrupt attached on D2."));
#endif

    // Clear any initial FIFO data
    mpu.resetFIFO();
    dmpDataReady = false;

    return true;
}

// ---------------------------------------------------------------------------
//  imuCalibrate()
//  Average CALIBRATION_SAMPLES pitch readings to find the "upright" angle.
// ---------------------------------------------------------------------------
void imuCalibrate() {
#if DEBUG_SERIAL
    Serial.println(F("Hold robot still for calibration..."));
#endif

    float sum = 0.0f;
    uint16_t count = 0;

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];  // yaw, pitch, roll

    while (count < CALIBRATION_SAMPLES) {
        if (dmpDataReady) {
            dmpDataReady = false;
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

                // ypr[1] is pitch in radians
                sum += ypr[1] * 180.0f / M_PI;
                count++;
            }
        }
    }

    balanceAngle = sum / (float)CALIBRATION_SAMPLES;

#if DEBUG_SERIAL
    Serial.print(F("Balance angle: "));
    Serial.print(balanceAngle, 2);
    Serial.println(F(" deg"));
#endif
}

// ---------------------------------------------------------------------------
//  imuUpdate()
//  Returns true if a new angle was read. Writes pitch (degrees) to *angle.
// ---------------------------------------------------------------------------
bool imuUpdate(float *angle) {
    // Check for FIFO overflow before reading
    uint8_t intStatus = mpu.getIntStatus();
    if (intStatus & 0x10) {  // FIFO overflow bit
        mpu.resetFIFO();
        return false;  // skip this cycle — data was corrupted
    }

    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        return false;
    }

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Convert pitch from radians to degrees
    // Multiply by 2 because the DMP returns half the actual pitch angle
    *angle = 2 * ypr[1] * 180.0f / M_PI;

    return true;
}

// ---------------------------------------------------------------------------
//  imuGetBalanceAngle()
// ---------------------------------------------------------------------------
float imuGetBalanceAngle() {
    return balanceAngle;
    // return 0.0f;  // --- IGNORE ---  // For testing, assume robot is perfectly upright
}
