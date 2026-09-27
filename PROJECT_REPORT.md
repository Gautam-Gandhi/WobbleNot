# Self-Balancing Two-Wheel Robot — Project Report

**Platform:** Arduino Nano (ATmega328P)  
**Firmware Language:** C++ (Arduino framework, PlatformIO)  
**Build System:** PlatformIO  
**Status:** ✅ Working

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware Components & Connections](#2-hardware-components--connections)
3. [Development Environment](#3-development-environment)
4. [System Architecture Overview](#4-system-architecture-overview)
5. [Firmware Modules — Deep Dive](#5-firmware-modules--deep-dive)
   - 5.1 [Configuration Layer — `config.h`](#51-configuration-layer--configh)
   - 5.2 [IMU Module — `imu.cpp / imu.h`](#52-imu-module--imucpp--imuh)
   - 5.3 [Motor Module — `motors.cpp / motors.h`](#53-motor-module--motorscpp--motorsh)
   - 5.4 [PID Controller — `pid.cpp / pid.h`](#54-pid-controller--pidcpp--pidh)
   - 5.5 [Main Control Loop — `main.cpp`](#55-main-control-loop--maincpp)
6. [Key Engineering Decisions & Challenges](#6-key-engineering-decisions--challenges)
7. [Control Theory — The Inverted Pendulum Problem](#7-control-theory--the-inverted-pendulum-problem)
8. [PID Tuning Methodology](#8-pid-tuning-methodology)
9. [Build Output & Resource Usage](#9-build-output--resource-usage)
10. [Results & Observations](#10-results--observations)

---

## 1. Project Overview

This project implements a **self-balancing two-wheeled robot** (an inverted pendulum on wheels) using an Arduino Nano as the sole microcontroller. The robot continuously reads its tilt angle from an MPU6050 inertial measurement unit (IMU), computes a correction signal using a PID controller, and drives two NEMA17 stepper motors via A4988 drivers to keep itself upright.

I started this as a personal hobby project. The goal was to build a functional balancing robot from scratch to deeply understand the underlying control theory and hardware integration. While I referenced various online resources and documentation for the math and hardware interfaces, the entire firmware architecture and codebase were written independently.

The core challenge is that a two-wheel robot is an **inherently unstable system**: without active control it will fall within fractions of a second. Achieving stability requires a control loop that is both fast (≥ 100 Hz) and deterministic, running entirely on a resource-constrained 8-bit microcontroller with only 2 KB of RAM and 30 KB of Flash.

### Goals

- **Primary:** Achieve stable, autonomous balancing on flat surfaces
- **Performance:** Control loop rate ≥ 200 Hz, angle measurement latency < 5 ms
- **Robustness:** Automatic fall detection, safe motor disable, and hands-free restart
- **Maintainability:** All tunable parameters in a single `config.h` header, zero magic numbers in source files

---

## 2. Hardware Components & Connections

### Component List

| Component | Specification |
|:----------|:-------------|
| Microcontroller | Arduino Nano (ATmega328P, 16 MHz, 2 KB RAM, 32 KB Flash) |
| IMU | MPU6050 (6-axis, I²C, onboard Digital Motion Processor) |
| Stepper Drivers | 2× A4988 (1/16 microstepping, shared ENABLE line) |
| Stepper Motors | 2× NEMA17 (1.8°/step, 200 steps/rev) |
| Microstepping | 1/16 → 3200 steps/rev effective resolution |

### Pin Assignment

| Function | Arduino Pin | ATmega328P Port |
|:---------|:------------|:----------------|
| MPU6050 INT (data ready) | D2 | PORTD.2 (INT0) |
| Motor A Left — STEP | D7 | PORTD.7 |
| Motor A Left — DIR | D3 | PORTD.3 |
| Motor B Right — STEP | D4 | PORTD.4 |
| Motor B Right — DIR | D5 | PORTD.5 |
| A4988 ENABLE (both, active-LOW) | D6 | PORTD.6 |
| MPU6050 SDA | A4 | (I²C hardware) |
| MPU6050 SCL | A5 | (I²C hardware) |

> All motor control signals (STEP, DIR, ENABLE for both motors) are deliberately placed on **PORTD**, enabling the ISR to manipulate all of them with a single 8-bit port write instead of multiple `digitalWrite()` calls — a critical performance optimisation.

---

## 3. Development Environment

### Toolchain

| Tool | Role |
|:-----|:-----|
| **PlatformIO** (VS Code extension) | Build system, dependency manager, uploader |
| **avr-gcc 7.3.0** | C++ compiler for ATmega328P |
| **avrdude** | Firmware upload via USB/serial bootloader |
| **electroniccats/MPU6050 @ ^1.3.1** | IMU + DMP library (managed by PlatformIO) |

### Project Configuration — `platformio.ini`

```ini
[env:nanoatmega328new]
platform  = atmelavr
board     = nanoatmega328new
framework = arduino
lib_deps  =
    electroniccats/MPU6050@^1.3.1
build_flags =
    -DMPU6050_DMP_FIFO_RATE_DIVISOR=0x00
monitor_speed = 115200
```

The `build_flags` entry is significant: `-DMPU6050_DMP_FIFO_RATE_DIVISOR=0x00` overrides the library's compile-time constant that controls the MPU6050 DMP output rate. The library default (`0x01`) produces 100 Hz; setting it to `0x00` maximises the DMP output to **200 Hz**, doubling the control loop bandwidth.

---

## 4. System Architecture Overview

The firmware is split into **five source modules**, each with a single clear responsibility:

```
┌────────────────────────────────────────────────────────┐
│                        main.cpp                        │
│   setup(): init sequence, calibration, PID seed        │
│   loop():  interrupt-driven control cycle              │
└────────┬──────────────┬──────────────┬─────────────────┘
         │              │              │
         ▼              ▼              ▼
    ┌─────────┐   ┌──────────┐   ┌──────────┐
    │ imu.cpp │   │ pid.cpp  │   │motors.cpp│
    │         │   │          │   │          │
    │ MPU6050 │   │ PID      │   │ Timer1   │
    │ DMP     │   │ compute  │   │ ISR step │
    │ pipeline│   │ filter   │   │ engine   │
    └─────────┘   └──────────┘   └──────────┘
         │                            │
         ▼                            ▼
    ┌─────────────────────────────────────┐
    │              config.h               │
    │  All tunable constants in one place │
    └─────────────────────────────────────┘
```

### Execution Flow

```
Power-on
   │
   ├─ imuInit()        → I²C @ 400kHz, DMP init, sensor auto-cal, interrupt on D2
   ├─ imuCalibrate()   → Average 100 DMP packets → store balance angle
   ├─ motorsInit()     → Configure PORTD DDR, Timer1 CTC @ 10kHz
   ├─ imuUpdate()      → Seed PID with first valid angle
   └─ pidReset(angle)  → Clear integral, derivative state

loop() [called at ~200 Hz, gated by DMP interrupt flag]
   │
   ├─ imuUpdate(&angle) → Read latest DMP packet, check FIFO overflow
   ├─ tilt check        → If |angle - balanceAngle| > 60°: fallen state machine
   ├─ pidCompute()      → P + I + D → signed output
   └─ setMotorSpeed()   → Map PID output to step interval, write atomically
                              │
                              └─ ISR(TIMER1_COMPA_vect) fires every 100µs
                                   → Decrement counters → pulse STEP pins
```

---

## 5. Firmware Modules — Deep Dive

### 5.1 Configuration Layer — `config.h`

All tunable constants live in a single header, compiled out when unused. This means changing a PID gain requires editing one line and re-uploading — no source file hunting.

**Key constants:**

| Constant | Value | Purpose |
|:---------|:------|:--------|
| `TIMER1_PERIOD_US` | 100 | ISR fires every 100 µs (10 kHz) |
| `MIN_STEP_INTERVAL` | 1 | Max motor speed: 1/(1×100µs) = **10,000 steps/s = 187 RPM** |
| `MAX_STEP_INTERVAL` | 500 | Min motor speed: 1/(500×100µs) = **20 steps/s** |
| `DMP_RATE_HZ` | 200 | DMP output rate; determines fixed PID timestep |
| `PID_KP` | 300.0 | Proportional gain |
| `PID_KI` | 0.0 | Integral gain (tuned to zero — no steady-state offset needed) |
| `PID_KD` | 1.0 | Derivative gain |
| `PID_OUTPUT_MAX` | 10000.0 | Clamps PID output; sets full-speed threshold |
| `PID_D_FILTER_ALPHA` | 0.5 | IIR low-pass coefficient on D-term |
| `MIN_PID_OUTPUT` | 10 | Dead-zone: motors stop below this PID magnitude |
| `FALLEN_ANGLE_DEG` | 60.0 | Safety cutoff angle |
| `CALIBRATION_SAMPLES` | 100 | DMP packets averaged to find balance angle |
| `DEBUG_SERIAL` | 0 | Compile-time switch: 0 = zero serial overhead |

The `DEBUG_SERIAL` flag is particularly important. When set to `0`, all `Serial.print()` calls are compiled out entirely via `#if DEBUG_SERIAL ... #endif` guards. This is critical because `Serial.print()` at 115200 baud is **blocking** — a typical debug line of ~40 characters takes approximately 3.5 ms, which at 200 Hz represents 70% of one control cycle.

---

### 5.2 IMU Module — `imu.cpp / imu.h`

The IMU module encapsulates all interaction with the MPU6050 sensor and its onboard Digital Motion Processor (DMP).

#### The MPU6050 DMP Pipeline

The MPU6050 contains a dedicated co-processor called the **Digital Motion Processor (DMP)**. Rather than streaming raw accelerometer and gyroscope data for the host to filter, the DMP internally runs a sensor fusion algorithm and outputs **quaternions** at a configurable rate — in this project, 200 Hz.

The DMP pipeline in `imuUpdate()` extracts a pitch angle in five steps:

```
Raw FIFO bytes (28 bytes per packet)
        │
        ▼  dmpGetQuaternion()
Quaternion q {w, x, y, z}   ← unit quaternion representing 3D orientation
        │
        ▼  dmpGetGravity()
gravity vector {gx, gy, gz}  ← direction of gravity in sensor frame
        │                      computed as: gx = 2(xz - wy)
        │                                   gy = 2(wx + yz)
        │                                   gz = w²- x²- y²+ z²
        ▼  dmpGetYawPitchRoll()
ypr[0] = yaw    (atan2 of quaternion components)
ypr[1] = pitch  (atan2(gx, sqrt(gy²+gz²)))  ← this is the tilt axis
ypr[2] = roll   (atan2(gy, gz))
        │
        ▼  × (2 × 180/π)
pitch angle in degrees
```

The output of `dmpGetYawPitchRoll()` is in **radians**. A correction factor of 2 is applied empirically to match physical tilt readings on this specific hardware mounting:

```cpp
*angle = 2 * ypr[1] * 180.0f / M_PI;
```

#### Sensor Auto-Calibration

During `imuInit()`, the library's built-in six-iteration calibration is called:

```cpp
mpu.CalibrateAccel(6);
mpu.CalibrateGyro(6);
```

This routine iteratively measures and corrects the sensor's zero-point offsets (the raw values output when the sensor is perfectly still and level). Six iterations converge the offsets to within ±1 LSB, stored in the sensor's internal registers. This is the low-level sensor calibration, distinct from the balance-angle calibration.

#### Balance Angle Calibration

After sensor offset calibration, `imuCalibrate()` finds the robot's **physical balance point** — the pitch angle corresponding to "perfectly upright on this specific robot with this specific weight distribution":

```cpp
void imuCalibrate() {
    float sum = 0.0f;
    uint16_t count = 0;
    while (count < CALIBRATION_SAMPLES) {
        if (dmpDataReady) {
            // ... read DMP packet ...
            sum += ypr[1] * 180.0f / M_PI;
            count++;
        }
    }
    balanceAngle = sum / (float)CALIBRATION_SAMPLES;
}
```

100 consecutive DMP readings are averaged. This eliminates the need to manually measure or guess the balance point — the robot self-calibrates every power-on, provided it is held still and upright during the first ~0.5 seconds of startup.

#### Interrupt-Driven Architecture

The MPU6050 INT pin (D2 / ATmega328P INT0) is connected to hardware external interrupt INT0. When the DMP finishes computing a new packet, it asserts the INT pin, triggering:

```cpp
static void dmpISR() {
    dmpDataReady = true;
}
attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpISR, RISING);
```

The ISR only sets a single `volatile bool` flag — it performs no I²C communication. The actual DMP packet read happens in the main loop, keeping the ISR latency under 1 µs and avoiding the critical issue of performing blocking I²C operations inside an interrupt context (I²C requires interrupts to be enabled to function, so calling Wire inside an ISR would deadlock).

#### FIFO Overflow Detection

The MPU6050's onboard FIFO buffer is 1024 bytes. At 200 Hz with a packet size of 28 bytes, the FIFO fills at 5600 bytes/sec. If the main loop ever stalls, the FIFO overflows and buffered data becomes corrupted. `imuUpdate()` checks the interrupt status register before every read:

```cpp
uint8_t intStatus = mpu.getIntStatus();
if (intStatus & 0x10) {   // bit 4 = FIFO overflow flag
    mpu.resetFIFO();
    return false;           // skip this cycle cleanly
}
```

On overflow: the FIFO is reset and the cycle skipped, rather than feeding corrupted quaternion data to the PID controller which would produce a violent spurious motor command.

---

### 5.3 Motor Module — `motors.cpp / motors.h`

The motor module implements a **hardware-timer-based step pulse generator** for two stepper motors driven by A4988 drivers.

#### A4988 Driver Protocol

The A4988 stepper driver requires two signals per motor:
- **STEP**: A rising edge causes the motor to advance by one microstep
- **DIR**: Logic level determines rotation direction
- **ENABLE**: Active-LOW; de-asserting this pin de-energises all coils

Timing constraints the firmware must satisfy:
- Minimum STEP pulse width: **1 µs** (high and low phases)
- DIR setup time before STEP rising edge: **200 ns**

#### Timer1 CTC Step Engine

Timer1 is configured in **Clear Timer on Compare (CTC) mode** with an 8× prescaler:

```cpp
TCCR1B = (1 << WGM12) | (1 << CS11);   // CTC mode, prescaler=8
OCR1A  = (16UL * TIMER1_PERIOD_US / 8) - 1;  // = 199 for 100µs
TIMSK1 = (1 << OCIE1A);                // enable compare-match interrupt
```

- Timer clock = 16 MHz / 8 = **2 MHz** (0.5 µs per tick)
- OCR1A = 199 → interrupt fires every 200 ticks × 0.5 µs = **100 µs**
- ISR fires at **10,000 Hz**

Inside the ISR, two independent countdown counters (`stepCounterL`, `stepCounterR`) decrement on every tick. When a counter reaches zero, a STEP pulse is generated and the counter reloads:

```cpp
ISR(TIMER1_COMPA_vect) {
    uint8_t stepMask = 0;

    if (stepIntervalL != 0 && --stepCounterL == 0) {
        stepMask |= STEP_L_MASK;
        stepCounterL = stepIntervalL;
    }
    if (stepIntervalR != 0 && --stepCounterR == 0) {
        stepMask |= STEP_R_MASK;
        stepCounterR = stepIntervalR;
    }

    if (stepMask) {
        PORTD |= stepMask;               // STEP HIGH (both simultaneously)
        /* 16 NOP instructions = 1µs at 16MHz */
        PORTD &= ~stepMask;              // STEP LOW
    }
}
```

**Key design points:**
- Both STEP pins are driven simultaneously with a single `PORTD |= stepMask` write, eliminating inter-motor timing skew — both wheels always step at the same instant
- 16 NOP instructions produce exactly 1 µs at 16 MHz, satisfying the A4988 minimum pulse width requirement
- The ISR completes in approximately 20 clock cycles (1.25 µs) — well within the 100 µs period

#### Speed Mapping — PID Output to Step Interval

The PID controller outputs a signed float `u`. `setMotorSpeed()` maps the magnitude to a step interval:

```
step_interval = PID_OUTPUT_MAX / |u|
```

This gives a linear relationship between PID output and actual wheel angular velocity:
```
step rate (steps/s) = |u| / (PID_OUTPUT_MAX × 100µs)
```

The interval is clamped to `[MIN_STEP_INTERVAL, MAX_STEP_INTERVAL]` = [1, 500], giving a wheel speed range of 20–10,000 steps/sec (0.375–187 RPM with 1/16 microstepping). The sign of `u` determines the DIR pin state.

#### Motor Dead-Zone

When the PID output magnitude is below `MIN_PID_OUTPUT` (= 10), the motors are stopped entirely. At these output levels the computed step interval would be enormous (≥ 1000 ticks = 100 ms between steps), producing no useful torque while consuming holding current and generating audible resonance noise.

#### Atomic Direction + Speed Updates

Both DIR pins and step intervals are updated inside a single `ATOMIC_BLOCK(ATOMIC_RESTORESTATE)`, which disables interrupts for the duration and restores the SREG register afterward. This serves two purposes:

1. Guarantees the A4988's 200 ns DIR setup time: DIR is written first, then the interval — the Timer1 ISR cannot fire a STEP between the two writes
2. Prevents the PID and ISR from seeing a partially-updated state (e.g., new interval with old DIR)

Additionally, the step counters are clamped immediately on speed change:
```cpp
if (stepCounterL == 0 || stepCounterL > intervalL) stepCounterL = intervalL;
```
This ensures a speed increase takes effect within at most 100 µs (one timer tick) rather than waiting for the old counter to expire naturally.

---

### 5.4 PID Controller — `pid.cpp / pid.h`

The PID controller translates the tilt angle error into a motor speed command. Three key implementation decisions differentiate this controller from a naive textbook implementation.

#### Fixed Timestep

```cpp
const float dt = 1.0f / (float)DMP_RATE_HZ;  // = 0.005 s at 200 Hz
```

Since the DMP produces packets at a crystal-controlled 200 Hz, using a constant `dt = 0.005 s` is not an approximation — it is the ground truth. The alternative (`micros()`-based dt) introduces 4 µs quantisation noise on every call and bakes any system timing jitter (Serial prints, I²C bus contention) into the D-term, causing noise spikes during irregular cycles. A fixed dt also makes PID gains fully deterministic — `Kd = 1.0` means the same physical damping regardless of CPU load.

#### Derivative on Measurement (Not Error)

A textbook PID computes the derivative of the **error**. For a constant setpoint this is equivalent, but if the setpoint changes (e.g., during a restart when `pidReset` seeds `prevAngle`), the error-based derivative produces a large instantaneous spike — **derivative kick**. The implementation uses the **measurement-based** derivative instead:

```cpp
float rawDerivative = -(currentAngle - prevAngle) / dt;
prevAngle = currentAngle;
```

The negative sign is because `error = angle - setpoint`, so `d(error)/dt = d(angle)/dt` for a constant setpoint — and negating the measurement derivative converts it to the error-derivative sign convention.

#### D-Term Low-Pass Filter

The raw numerical derivative amplifies high-frequency sensor noise. A first-order IIR low-pass filter smooths this before scaling by Kd:

```cpp
filteredDerivative = PID_D_FILTER_ALPHA * filteredDerivative
                   + (1.0f - PID_D_FILTER_ALPHA) * rawDerivative;
```

With `PID_D_FILTER_ALPHA = 0.5` and `dt = 0.005 s`, the filter cutoff frequency is approximately:

```
f_c ≈ (1 - alpha) / (2π × alpha × dt)  ≈  32 Hz
```

Frequencies above ~32 Hz (sensor noise, vibration) are attenuated; frequencies below ~32 Hz (actual robot motion, typically < 5 Hz for a well-tuned bot) pass through unattenuated.

#### Integral Anti-Windup

```cpp
float iMax = PID_OUTPUT_MAX / (PID_KI + 0.001f);
if (integral >  iMax) integral =  iMax;
if (integral < -iMax) integral = -iMax;
```

The integral is clamped to prevent windup — where the integrator accumulates a large value while the robot is fallen over (when the error cannot be corrected), causing an enormous motor command on restart. The `+0.001f` term prevents division by zero when `PID_KI = 0`, without affecting behaviour when Ki has a real value.

#### Initialization — No First-Call Spike

`pidReset(float initialAngle)` is called with the live current angle before motors are enabled. This seeds `prevAngle = initialAngle`, so the first derivative computation produces:

```
rawDerivative = -(initialAngle - initialAngle) / dt = 0
```

A naive `pidReset()` initialising `prevAngle = 0.0f` would produce a first-call derivative of e.g. `-(5.0° - 0.0°) / 0.005s = -1000 units`, which multiplied by Kd and mapped to motor speed would cause an instant violent lurch.

#### Complete `pidCompute()` Summary

```
error = currentAngle - targetAngle

P = Kp × error
I = Ki × clamp(∫error×dt, ±iMax)
D = Kd × LPF(-(angle[n] - angle[n-1]) / dt)

output = clamp(P + I + D, ±PID_OUTPUT_MAX)
```

---

### 5.5 Main Control Loop — `main.cpp`

#### `setup()` — Initialisation Sequence

The initialisation order is deliberate:

| Step | Call | Why this order |
|:-----|:-----|:---------------|
| 1 | `imuInit()` | I²C must be configured first. Starts DMP, runs sensor auto-calibration, attaches INT0 interrupt |
| 2 | `imuCalibrate()` | Motors must be OFF during calibration to avoid vibration corrupting the averaged angle |
| 3 | `motorsInit()` | Timer1 ISR enabled. Done after calibration to prevent motor vibration during averaging |
| 4 | `imuUpdate(&initAngle)` | Blocks until first valid DMP packet — seeds PID's `prevAngle` |
| 5 | `pidReset(initAngle)` | Sets `prevAngle = initAngle` so first derivative is zero, not a spike |
| 6 | LED blink ×5 | Visual "ready" signal — confirms calibration complete, robot can be released |

#### `loop()` — The Control Cycle

The main loop is **interrupt-gated**: it returns immediately until `dmpDataReady` is set by the DMP ISR on D2. This means the loop body runs at exactly the DMP rate (200 Hz) without any additional timing logic or busy-waiting:

```cpp
void loop() {
    if (!dmpDataReady) return;          // fast exit if no new data
    dmpDataReady = false;

    float angle;
    if (!imuUpdate(&angle)) return;     // skip on FIFO overflow

    float tiltError = angle - imuGetBalanceAngle();

    if (abs(tiltError) > FALLEN_ANGLE_DEG) { /* fallen state machine */ }

    float pidOutput = pidCompute(angle, balanceAngle);
    setMotorSpeed((int16_t)pidOutput, (int16_t)pidOutput);
}
```

Both motors receive the same speed command because pure balancing requires symmetric wheel motion. The independent `speedL`/`speedR` interface in `setMotorSpeed()` is left as a hook for future differential steering.

#### Fallen State Machine

When tilt exceeds `FALLEN_ANGLE_DEG` (60°), the robot is clearly down. The state machine:

1. `motorsDisable()` — stops all stepping, de-energises coils (motors go free)
2. Enters a blocking inner loop blinking the built-in LED
   - Blink period 300 ms when fallen; 50 ms when held near vertical (within `RESTART_ANGLE_DEG = 5°`)
   - This visual feedback guides the user: slow blink = fallen, fast blink = hold it here
3. Tracks a continuous 2-second upright hold timer (`RESTART_HOLD_TIME_MS = 2000`)
4. On 2-second completion:
   - `imuUpdate(&angle)` — drain any stale FIFO packets, get fresh angle
   - `pidReset(angle)` — seeds PID `prevAngle` with real current angle
   - `motorsEnable()` — re-energises drivers
   - LED off

The ordering of `pidReset` before `motorsEnable` is intentional: any derivative spike from stale `prevAngle` occurs while motors are still disabled, rather than causing a lurch at the moment of re-enable.

---

## 6. Key Engineering Decisions & Challenges

### Challenge 1: Timer1 ISR CPU Budget

**Problem:** An early configuration used `TIMER1_PERIOD_US = 10` (100 kHz ISR rate). The ISR consumed roughly 12–15% of CPU bandwidth. More critically, it interrupted the MPU6050 I²C reads (which take ~560 µs at 400 kHz, reading 28 bytes) approximately 56 times per read — causing I²C glitches and corrupted quaternion data.

**Solution:** `TIMER1_PERIOD_US = 100` (10 kHz). Maximum motor speed **preserved** by setting `MIN_STEP_INTERVAL = 1`: 1 tick × 100 µs = 10,000 steps/s — identical to the previous maximum. CPU overhead from the ISR dropped from ~15% to ~1.5%.

### Challenge 2: Step Counter Reload Latency

**Problem:** The original reload condition `if (stepCounterL == 0) stepCounterL = intervalL` only reloaded when the counter happened to be exactly zero. A speed increase (interval 200 → 50) left the counter at e.g. 180, causing up to 18 ms of latency before the new speed took effect — during which the robot was still pushing with the old motor force.

**Solution:** `if (stepCounterL == 0 || stepCounterL > intervalL) stepCounterL = intervalL`. The counter is clamped immediately on any speed increase, giving worst-case response latency of 100 µs (one timer tick).

### Challenge 3: DIR Pin Race Condition

**Problem:** DIR pins were written outside the `ATOMIC_BLOCK`. The Timer1 ISR could fire a STEP pulse between the DIR write and the interval update, causing one step in the wrong direction.

**Solution:** DIR pin writes moved inside the `ATOMIC_BLOCK`, atomically bundled with the interval update. Interrupts are disabled for the few hundred nanoseconds of the entire sequence.

### Challenge 4: DMP Rate Maximisation Without Library Modification

**Problem:** The electroniccats MPU6050 library hardcodes `MPU6050_DMP_FIFO_RATE_DIVISOR = 0x01` (100 Hz DMP output) as a compile-time `#ifndef` constant inside the library `.cpp` file. Modifying the library directly would break on library updates.

**Solution:** Override at build time via `platformio.ini` build flag: `-DMPU6050_DMP_FIFO_RATE_DIVISOR=0x00`. Because the library uses `#ifndef`, the compiler uses our definition and the library's default is never compiled. No library files modified.

### Challenge 5: Derivative Kick on Startup and Restart

**Problem:** The original `pidReset()` initialised `prevAngle = 0.0f`. The first `pidCompute()` call computed `-(currentAngle - 0.0f) / 0.005s` — at a typical startup tilt of 5°, this produced a spike of −1000 units. Multiplied by even a small Kd, this created a violent motor lurch the instant the robot was released.

**Solution:** `pidReset(float initialAngle)` seeds `prevAngle = initialAngle`. The first derivative is `-(initialAngle - initialAngle) / dt = 0.0`. Applied both at startup and after every fall recovery.

---

## 7. Control Theory — The Inverted Pendulum Problem

A two-wheeled self-balancing robot is a physical realisation of the **inverted pendulum on a cart** — a classic problem in control theory. The upright equilibrium is an **unstable fixed point**: any small perturbation grows exponentially without active control, with a natural time constant of approximately:

```
τ = sqrt(L / g)   where L = effective pendulum length to CoG
```

For a robot with CoG at ~20 cm height: τ ≈ 0.14 s. The robot falls to ~45° in under 200 ms, which sets a hard requirement on the minimum control loop frequency.

### State Space

The system has four state variables: tilt angle `θ`, angular velocity `θ̇`, wheel position `x`, and wheel velocity `ẋ`. Without wheel encoders, only `θ` and `θ̇` are observable. This means positional drift (the robot slowly walking forward/backward while balancing) cannot be corrected — a known limitation addressed by adding encoders in future work.

### PID as a Damped Restoring Controller

For small angles (linearised around upright equilibrium), the PID controller acts as:

```
u = Kp × θ  +  Ki × ∫θ dt  +  Kd × θ̇

where:
  Kp × θ   = restoring torque proportional to tilt (stiffness)
  Kd × θ̇  = velocity damping (shock absorber — the critical stability term)
  Ki × ∫θ  = slow correction of steady-state lean
```

The P and D terms alone are sufficient to stabilise the upright equilibrium. Without Kd the system has zero damping and oscillates indefinitely at the natural pendulum frequency. The I term corrects systematic lean from asymmetric weight distribution or calibration error — in practice, with good calibration it can be left at zero.

---

## 8. PID Tuning Methodology

Tuning was performed sequentially, building on each step:

1. **Motor direction verification**: With `Ki = Kd = 0` and a small `Kp = 100`, verified that wheels spin in the direction of tilt (catching the fall, not accelerating it).

2. **Finding Kp_critical**: `Kp` was doubled repeatedly (10 → 20 → 40 → 80 → 160 → 320) until the robot exhibited sustained constant-amplitude oscillation. This is `Kp_critical`. Operating gain set to `0.6 × Kp_critical`.

3. **Adding derivative damping**: `Kd` increased from 0 in small increments until oscillations were damped to the point that the robot made brief balance attempts. Then increased until stable balance was achieved.

4. **D-filter adjustment**: `PID_D_FILTER_ALPHA = 0.5` was found to provide a good trade-off between noise rejection and response speed.

5. **Dead-zone tuning**: `MIN_PID_OUTPUT = 10` was found to eliminate idle motor noise without introducing noticeable latency for small corrections.

6. **Ki**: Not required. The balance angle calibration was accurate enough that no steady-state offset was observed.

### Final Tuned Values

| Parameter | Value | Notes |
|:----------|:------|:------|
| `PID_KP` | 300.0 | ~0.6 × Kp_critical |
| `PID_KI` | 0.0 | Not needed |
| `PID_KD` | 1.0 | Provides sufficient damping at 200 Hz |
| `PID_OUTPUT_MAX` | 10000.0 | Full speed at ~33° tilt |
| `PID_D_FILTER_ALPHA` | 0.5 | ~32 Hz D-term cutoff |
| `MIN_PID_OUTPUT` | 10 | Stops motors when nearly balanced |

---

## 9. Build Output & Resource Usage

```
Platform:  Atmel AVR (atmelavr 5.3.0)
Board:     Arduino Nano ATmega328 (New Bootloader)
Framework: Arduino
Compiler:  avr-gcc 7.3.0

RAM:   [===       ]  27.9%   (used 571 bytes from 2048 bytes)
Flash: [======    ]  60.3%   (used 18522 bytes from 30720 bytes)

Build result: SUCCESS
Build time:   3.2 seconds
```

**RAM allocation breakdown:**
- DMP packet buffer: 64 bytes
- Wire I²C TX/RX buffers: 2 × 32 bytes
- PID state (`prevAngle`, `integral`, `filteredDerivative`): 12 bytes
- Timer1 ISR state (`stepCounterL/R`, `stepIntervalL/R`): 8 bytes
- Balance angle, stack, Arduino runtime: remainder

72% of RAM remains free — comfortable headroom for future features like Bluetooth command parsing buffers.

**Flash allocation:** The largest consumer is the DMP firmware blob embedded in the library (~14 KB of on-chip program memory). Floating-point math routines (`atan2`, division, `sqrt`) account for several KB. 39% of Flash remains free.

---

## 10. Results & Observations

The robot achieves stable autonomous balancing on flat hard surfaces.

- **Startup time:** ~6–8 seconds (I²C init + DMP firmware load + sensor calibration + 100-sample balance angle averaging at 200 Hz)
- **Control rate:** 200 Hz, verified by monitoring the DMP interrupt rate
- **Disturbance rejection:** Recovers from gentle pushes (5–10° perturbation) within approximately 0.5–1.0 seconds
- **Integral term:** Not required — `PID_KI = 0` throughout. The startup calibration produces an accurate enough balance angle to eliminate steady-state offset
- **Fallen state machine:** Reliable in testing. Blink frequency correctly distinguishes fallen vs. upright-ready states. Restart produces no initial lurch due to the `pidReset(angle)` seeding

### Potential Future Improvements

| Feature | Benefit |
|:--------|:--------|
| Wheel encoders + position loop | Eliminate translational drift while balancing |
| Bluetooth remote control | Accept velocity/heading commands to drive the robot while balancing |
| Gyro feedforward | Use raw DMP gyro data directly as a feedforward term, reducing dependence on the noisy numerical derivative |
| Sensor offset persistence | Store `CalibrateAccel/Gyro` results in EEPROM to skip re-calibration on every power-on |
| Terrain adaptation | Closed-loop wheel velocity control for performance on uneven surfaces |

---

*End of Report*
