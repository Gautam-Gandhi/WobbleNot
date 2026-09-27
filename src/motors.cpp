#include "motors.h"
#include "config.h"
#include <util/atomic.h>

// ---------------------------------------------------------------------------
//  ISR-shared state (volatile)
// ---------------------------------------------------------------------------

// Step counters: decremented each ISR tick. When reaching 0, a step pulse
// is generated and the counter is reloaded from stepInterval*.
// A stepInterval of 0 means "motor stopped".
static volatile uint16_t stepCounterL = 0;
static volatile uint16_t stepCounterR = 0;
static volatile uint16_t stepIntervalL = 0;  // 0 = stopped
static volatile uint16_t stepIntervalR = 0;  // 0 = stopped

// ---------------------------------------------------------------------------
//  Timer1 Compare-Match A ISR — Step Pulse Engine
//  Fires every TIMER1_PERIOD_US microseconds.
// ---------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect) {
    uint8_t stepMask = 0;

    // ---- Left motor ----
    if (stepIntervalL != 0) {
        if (--stepCounterL == 0) {
            stepMask |= STEP_L_MASK;
            stepCounterL = stepIntervalL;
        }
    }

    // ---- Right motor ----
    if (stepIntervalR != 0) {
        if (--stepCounterR == 0) {
            stepMask |= STEP_R_MASK;
            stepCounterR = stepIntervalR;
        }
    }

    // Generate simultaneous step pulses for both motors
    if (stepMask) {
        PORTD |= stepMask;   // Both STEP pins HIGH together
        __asm__ __volatile__(
            "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
            "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
            "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
            "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
        );
        PORTD &= ~stepMask;  // Both STEP pins LOW together
    }
}

// ---------------------------------------------------------------------------
//  motorsInit()
// ---------------------------------------------------------------------------
void motorsInit() {
    // Set motor pins as outputs using DDR registers (PORTD bits 3,4,5,6,7)
    DDRD |= STEP_L_MASK | STEP_R_MASK | DIR_L_MASK | DIR_R_MASK | (1 << 6);

    // All motor pins LOW initially
    PORTD &= ~(STEP_L_MASK | STEP_R_MASK | DIR_L_MASK | DIR_R_MASK);

    // Enable drivers (ENABLE pin LOW = active)
    PORTD &= ~(1 << 6);

    // ---- Configure Timer1 in CTC mode ----
    // Clock = 16 MHz, Prescaler = 8 → Timer clock = 2 MHz (0.5 µs per tick)
    // For 100 µs period: OCR1A = (2 MHz * 100 µs) - 1 = 199
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // CTC mode (WGM12), prescaler = 8 (CS11)
    TCCR1B = (1 << WGM12) | (1 << CS11);

    // Compare match value for desired period
    OCR1A = (16UL * TIMER1_PERIOD_US / 8) - 1;  // = 199 for 100 µs

    // Enable Timer1 Compare Match A interrupt
    TIMSK1 = (1 << OCIE1A);
    sei();
}

// ---------------------------------------------------------------------------
//  setMotorSpeed()
//  speedL, speedR: signed int16.
//     Positive = forward (direction to catch a forward tilt).
//     Negative = backward.
//     Magnitude is mapped to step interval:
//       higher magnitude → shorter interval → faster stepping.
//     Zero = stop.
// ---------------------------------------------------------------------------
void setMotorSpeed(int16_t speedL, int16_t speedR) {
    // ---- Direction ----
    bool dirL = (speedL >= 0);
    bool dirR = (speedR >= 0);

    // Apply inversion macros
#if INVERT_LEFT_MOTOR
    dirL = !dirL;
#endif
#if INVERT_RIGHT_MOTOR
    dirR = !dirR;
#endif

    // ---- Speed → step interval ----
    uint16_t absL = (uint16_t)abs(speedL);
    uint16_t absR = (uint16_t)abs(speedR);

    uint16_t intervalL, intervalR;

    // Dead-zone: if PID output is too small, stop the motor
    if (absL < MIN_PID_OUTPUT) {
        intervalL = 0;  // stopped
    } else {
        intervalL = (uint16_t)((float)PID_OUTPUT_MAX / (float)absL);
        if (intervalL < MIN_STEP_INTERVAL) intervalL = MIN_STEP_INTERVAL;
        if (intervalL > MAX_STEP_INTERVAL) intervalL = MAX_STEP_INTERVAL;
    }

    if (absR < MIN_PID_OUTPUT) {
        intervalR = 0;
    } else {
        intervalR = (uint16_t)((float)PID_OUTPUT_MAX / (float)absR);
        if (intervalR < MIN_STEP_INTERVAL) intervalR = MIN_STEP_INTERVAL;
        if (intervalR > MAX_STEP_INTERVAL) intervalR = MAX_STEP_INTERVAL;
    }

    // ---- Atomic write to ISR-shared variables ----
    // DIR pins are set inside the atomic block to guarantee the A4988's
    // 200ns DIR setup time is met before any STEP pulse from the ISR.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Set DIR pins
        if (dirL) {
            PORTD |= DIR_L_MASK;
        } else {
            PORTD &= ~DIR_L_MASK;
        }

        if (dirR) {
            PORTD |= DIR_R_MASK;
        } else {
            PORTD &= ~DIR_R_MASK;
        }

        stepIntervalL = intervalL;
        stepIntervalR = intervalR;

        // Cap counters to new interval so speed changes take effect promptly.
        // If counter exceeds new interval, clamp it; if motor was stopped, reload.
        if (intervalL != 0) {
            if (stepCounterL == 0 || stepCounterL > intervalL) stepCounterL = intervalL;
        }
        if (intervalR != 0) {
            if (stepCounterR == 0 || stepCounterR > intervalR) stepCounterR = intervalR;
        }
    }
}

// ---------------------------------------------------------------------------
//  motorsEnable() / motorsDisable()
// ---------------------------------------------------------------------------
void motorsEnable() {
    PORTD &= ~(1 << 6);  // ENABLE LOW = drivers active
}

void motorsDisable() {
    // Stop step generation first
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        stepIntervalL = 0;
        stepIntervalR = 0;
    }
    PORTD |= (1 << 6);   // ENABLE HIGH = drivers disabled
}
