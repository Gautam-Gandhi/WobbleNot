#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
//  Pin Definitions (ATmega328P — all motor pins on PORTD)
// =============================================================================

// MPU6050 interrupt (INT0 on D2)
#define MPU_INT_PIN       2

// Motor A (Left)
#define STEP_L_PIN        7   // PORTD.7
#define DIR_L_PIN         3   // PORTD.3

// Motor B (Right)
#define STEP_R_PIN        4   // PORTD.4
#define DIR_R_PIN         5   // PORTD.5

// A4988 shared enable (active LOW)
#define ENABLE_PIN        6   // PORTD.6

// PORTD bitmasks for direct port manipulation in ISR
#define STEP_L_MASK       (1 << 7)  // D7 = PORTD bit 7
#define STEP_R_MASK       (1 << 4)  // D4 = PORTD bit 4
#define DIR_L_MASK        (1 << 3)  // D3 = PORTD bit 3
#define DIR_R_MASK        (1 << 5)  // D5 = PORTD bit 5

// =============================================================================
//  Motor Direction Inversion
//  Set to true if a motor spins the wrong way during balancing.
// =============================================================================
#define INVERT_LEFT_MOTOR   false
#define INVERT_RIGHT_MOTOR  false

// =============================================================================
//  Stepper Configuration
// =============================================================================
#define MICROSTEPS        16
#define STEPS_PER_REV     (200 * MICROSTEPS)  // 3200 steps/rev

// =============================================================================
//  Timer1 Step Engine Configuration
// =============================================================================
// Timer1 ISR period in microseconds.
// With prescaler=8 and 16 MHz clock: OCR1A = (16 * TIMER1_PERIOD_US) / 8 - 1
#define TIMER1_PERIOD_US  100

// Speed limits (step interval in Timer1 ticks)
// Smaller interval = faster speed
#define MIN_STEP_INTERVAL 1     // fastest: 1/(1*100µs) = 10,000 steps/s
#define MAX_STEP_INTERVAL 500   // slowest: 1/(500*100µs) = 20 steps/s

// =============================================================================
//  DMP Configuration
// =============================================================================
// DMP output rate in Hz (set via MPU6050_DMP_FIFO_RATE_DIVISOR build flag)
// With divisor=0x00: 200Hz.  With divisor=0x01: 100Hz.
#define DMP_RATE_HZ       200

// =============================================================================
//  PID Tuning Constants
//  Adjust these and re-upload to tune the balance behaviour.
// =============================================================================
#define PID_KP            300.0f
#define PID_KI            0.0f
#define PID_KD            1.0f

// Maximum PID output magnitude (maps to motor speed)
#define PID_OUTPUT_MAX    10000.0f

// Low-pass filter coefficient for D-term (0.0 = no filtering, higher = more smoothing)
#define PID_D_FILTER_ALPHA  0.5f

// Minimum PID output magnitude to drive motors (dead-zone threshold)
// Below this, motors are stopped to avoid useless dithering
#define MIN_PID_OUTPUT    10

// Angle beyond which the robot is considered "fallen" and motors are killed
#define FALLEN_ANGLE_DEG  60.0f

// Auto-restart: robot must be held within this angle of balance to restart after falling
#define RESTART_ANGLE_DEG      5.0f
// Time (ms) robot must be held upright before motors re-enable
#define RESTART_HOLD_TIME_MS   2000

// =============================================================================
//  Calibration
// =============================================================================
// Number of DMP samples to average during balance-angle calibration
#define CALIBRATION_SAMPLES  100

// =============================================================================
//  Debug Serial Output
//  Set to 1 to enable debug prints (~4 Hz). Set to 0 for zero Serial overhead.
//  When disabled, the Serial code is compiled out entirely.
// =============================================================================
#define DEBUG_SERIAL      0
#define DEBUG_PRINT_INTERVAL  50  // print every N-th PID cycle (~200/50 = 4 Hz)

#endif // CONFIG_H
