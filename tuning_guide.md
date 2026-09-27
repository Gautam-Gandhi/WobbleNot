# Balance Bot — Complete Tuning Guide

This guide covers tuning **every** adjustable parameter in your firmware, not just Kp/Ki/Kd. Follow it sequentially — each step builds on the previous one.

---

## Table of Contents

1. [Pre-Tuning Hardware Checklist](#1-pre-tuning-hardware-checklist)
2. [Motor Direction Verification](#2-motor-direction-verification)
3. [Balance Angle Calibration](#3-balance-angle-calibration)
4. [Understanding What Each Parameter Does](#4-understanding-what-each-parameter-does)
5. [Step 1: Set Starting Values](#5-step-1-set-starting-values)
6. [Step 2: Find PID_OUTPUT_MAX](#6-step-2-find-pid_output_max)
7. [Step 3: Tune Kp (Proportional)](#7-step-3-tune-kp-proportional)
8. [Step 4: Tune Kd (Derivative)](#8-step-4-tune-kd-derivative)
9. [Step 5: Tune PID_D_FILTER_ALPHA](#9-step-5-tune-pid_d_filter_alpha)
10. [Step 6: Tune MIN_PID_OUTPUT (Dead-Zone)](#10-step-6-tune-min_pid_output-dead-zone)
11. [Step 7: Tune Ki (Integral) — Maybe](#11-step-7-tune-ki-integral--maybe)
12. [Step 8: Fine-Tune MAX_STEP_INTERVAL](#12-step-8-fine-tune-max_step_interval)
13. [Step 9: Tune FALLEN_ANGLE_DEG](#13-step-9-tune-fallen_angle_deg)
14. [Symptom → Fix Reference Table](#symptom--fix-reference-table)
15. [Recommended Starting Config](#recommended-starting-config)

---

## 1. Pre-Tuning Hardware Checklist

Before touching any software parameter, verify these hardware conditions. **A hardware problem will make software tuning impossible.**

- [ ] **Wheels are tight on motor shafts** — any slop between the motor shaft and wheel wastes the first few degrees of motor correction, creating a dead zone that no PID can compensate for
- [ ] **MPU6050 is rigidly mounted** — any flex, vibration isolation, or loose mounting causes sensor noise that the D-term amplifies into motor jitter
- [ ] **Robot's center of gravity is as high as possible** — a higher CoG gives a slower fall rate, making the control problem easier. Battery on top > battery on bottom
- [ ] **Both wheels touch the ground evenly** — if the robot leans to one side mechanically, it will never balance because the PID is fighting a mechanical offset
- [ ] **Wiring is tidy and secured** — loose wires shift the CoG during balancing, creating unpredictable disturbances
- [ ] **Battery is fully charged** — low battery = low stepper torque = the bot falls before the motors can catch it
- [ ] **A4988 current limit is set correctly** — too low = motors skip steps under load; too high = motors overheat and lose torque. Set the Vref potentiometer for ~70-80% of your NEMA17's rated current

> [!CAUTION]
> If any of the above are not satisfied, **stop and fix the hardware first**. No amount of PID tuning can compensate for mechanical slop, loose sensors, or insufficient motor torque.

---

## 2. Motor Direction Verification

This must be verified **before** any PID tuning. If the motors spin the wrong direction, the robot will actively accelerate its fall instead of correcting it.

### Test procedure

1. Set `DEBUG_SERIAL = 1` in [config.h](file:///c:/Users/gandh/OneDrive%20-%20IIIT%20Hyderabad/Coding/Robotics/balance-bot-3/balance-bot/src/config.h)
2. Set `PID_KP = 100`, `PID_KI = 0`, `PID_KD = 0`
3. Upload and open Serial Monitor
4. Hold the robot in your hand, tilted ~15° **forward** (nose down)
5. Watch the serial output — the angle should be **positive** (or negative — note which)
6. Watch the wheels — they should spin to drive the robot **forward** (the same direction it's tilting), as if trying to "catch" the fall by moving the base under the CoG

### If the wheels spin the wrong direction

**Case A: Both wheels spin backward when tilting forward**
→ Your angle sign is inverted relative to your motor direction. Options:
- Swap **both** `INVERT_LEFT_MOTOR` and `INVERT_RIGHT_MOTOR` to `true`
- Or physically swap the wiring on **both** motors' coil pairs (swap A1↔A2 on both motors)

**Case B: One wheel spins forward, the other backward**
→ One motor is wired in reverse. Swap the `INVERT_*_MOTOR` flag for the incorrect one only, OR physically swap that motor's coil wires.

**Case C: Both wheels spin the correct direction, but the robot still falls**
→ Direction is fine. The issue is Kp being too low or wrong. Move on to tuning.

> [!IMPORTANT]
> If you're unsure which direction is "forward", place the robot on the ground and tilt it. The wheels should spin toward the direction of tilt. Think of it like balancing a broomstick on your palm — your hand moves in the direction the broomstick is falling.

---

## 3. Balance Angle Calibration

The calibration routine averages 100 DMP readings at startup to find the "upright" angle. This sets `balanceAngle`, and the PID tries to keep the robot at this angle.

### Getting a good calibration

1. Power on the robot while holding it **perfectly still** and **perfectly upright** (the natural balance point, where it would stand if you let go on a perfectly flat surface)
2. Don't move during the `"Hold robot still for calibration..."` message
3. The Serial output will print `"Balance angle: X.XX deg"` — this is your calibration result

### If the balance angle seems wrong

**Symptom**: The robot consistently leans to one side when balancing, even with good PID values

**Fixes**:
- **Increase `CALIBRATION_SAMPLES`** from 100 to 200 or 300. More samples = more averaging = more accurate calibration. Takes longer at startup, but only happens once.
- **Manual offset**: If the robot always leans ~2° forward, you know the calibration is 2° off. You can add a manual trim by modifying the balance angle after calibration. (We can add a `BALANCE_ANGLE_TRIM` constant if needed.)
- **Check your surface**: Calibrate on a flat, level surface. A tilted table will bake an offset into the calibration.

---

## 4. Understanding What Each Parameter Does

Before tuning, understand what you're adjusting. Here's every tunable parameter and its physical effect:

### PID Gains

| Parameter | What it does | Physical effect |
|:----------|:-------------|:----------------|
| `PID_KP` | Multiplies the angle error directly | Higher = stronger, faster correction. Too high = violent oscillation. Too low = falls over before correcting. |
| `PID_KI` | Multiplies the accumulated error over time | Corrects persistent lean (steady-state error). Too high = slow, growing oscillations that build up. Almost always should be very small or zero. |
| `PID_KD` | Multiplies the rate of angle change | Dampens oscillations. Predicts where the angle is going and acts early. Too high = motor jitter/vibration from noise amplification. Too low = underdamped oscillation. |

### PID Support Parameters

| Parameter | What it does | Effect of too high | Effect of too low |
|:----------|:-------------|:-------------------|:------------------|
| `PID_OUTPUT_MAX` | Caps the maximum PID output sent to motors | Wastes resolution — PID output rarely reaches the cap, so you're using a small fraction of the output range. No real harm, but makes Kp look smaller. | Clips the PID output during large tilts — motor can't reach full speed to catch a fast fall. Robot falls over on disturbances. |
| `PID_D_FILTER_ALPHA` | Smoothing on D-term (0.0 = none, 0.9 = heavy) | Makes D-term slow to react — defeats the purpose of derivative. Robot oscillates as if Kd is too low. | D-term is noisy — motors jitter/vibrate even when the robot is nearly balanced. |
| `MIN_PID_OUTPUT` | Dead-zone — if `|pidOutput| < this`, motors stop | Ignores real tilts — robot won't correct small angles and slowly drifts then falls. | Motors dither uselessly at very slow speeds that produce no torque, wasting power and making noise. |

### Motor Parameters

| Parameter | What it does |
|:----------|:-------------|
| `MAX_STEP_INTERVAL` | Slowest motor speed (largest interval between steps). 500 × 100µs = 50ms between steps = 20 steps/s = 0.375 RPM. This is the minimum speed the motor will ever run at. |
| `MIN_STEP_INTERVAL` | Fastest motor speed. 1 × 100µs = 10,000 steps/s = 187 RPM. This caps the maximum wheel speed. |
| `FALLEN_ANGLE_DEG` | Angle threshold for "the robot has fallen over, kill motors". |

---

## 5. Step 1: Set Starting Values

Before tuning PID, set these parameters to safe starting values:

```cpp
// in config.h

#define PID_KP            0.0f      // we'll increase this first
#define PID_KI            0.0f      // leave at zero
#define PID_KD            0.0f      // leave at zero

#define PID_OUTPUT_MAX    5000.0f   // start moderate
#define PID_D_FILTER_ALPHA  0.5f   // moderate filtering
#define MIN_PID_OUTPUT    20        // small dead-zone

#define FALLEN_ANGLE_DEG  45.0f    // safety cutoff
#define MAX_STEP_INTERVAL 500      // 20 steps/s minimum speed
#define MIN_STEP_INTERVAL 1        // 10,000 steps/s maximum speed

#define DEBUG_SERIAL      0        // OFF for ground testing!
```

> [!WARNING]
> **Always use `DEBUG_SERIAL = 0` during ground balance testing.** Serial prints take 3-4ms each and create timing hiccups that can destabilize the robot. Only enable serial when the robot is held in your hand or on a test stand.

---

## 6. Step 2: Find PID_OUTPUT_MAX

`PID_OUTPUT_MAX` determines how the PID output maps to motor speed. Getting this right ensures you're using the full motor speed range effectively.

### How to choose it

The relationship is: `step_interval = PID_OUTPUT_MAX / |pidOutput|`

At maximum PID output (`pidOutput = PID_OUTPUT_MAX`), step_interval = 1 = maximum motor speed.

**The rule**: `PID_OUTPUT_MAX` should be set so that when the robot is tilted at a moderate recovery angle (say 15-20°), the PID output is high enough to drive the motors at near-full speed.

### Practical approach

1. Set `PID_KP = 100`, other gains zero
2. Enable `DEBUG_SERIAL = 1`, hold robot in hand
3. Tilt 20° and read the PID output from serial
4. The PID output at 20° tilt = `Kp × 20 = 2000`
5. With `PID_OUTPUT_MAX = 5000`, the step interval at this tilt = `5000/2000 = 2` → very fast stepping ✅
6. If the PID output at a moderate tilt is much smaller than `PID_OUTPUT_MAX`, the motors will barely move. Reduce `PID_OUTPUT_MAX`.

**Simple formula**: `PID_OUTPUT_MAX ≈ PID_KP × 30` is a reasonable starting point. This ensures the motor reaches full speed at about 30° of tilt.

> [!TIP]
> Don't overthink this parameter. Start with `5000` and adjust if:
> - Motors feel sluggish at moderate tilts → decrease PID_OUTPUT_MAX
> - Motors hit max speed at tiny tilts → increase PID_OUTPUT_MAX

---

## 7. Step 3: Tune Kp (Proportional)

**This is the most important step.** Kp determines the basic reaction strength.

### Procedure

1. Set `PID_KI = 0`, `PID_KD = 0`, `DEBUG_SERIAL = 0`
2. Start with `PID_KP = 10`
3. Place robot on the ground and release
4. **Observe the behavior** and match it to the table below:

| Observation | What's happening | Action |
|:------------|:-----------------|:-------|
| Robot slowly falls over, motors barely react | Kp way too low | **Double Kp** (10 → 20 → 40 → 80...) |
| Robot reacts, motors spin, but still falls (can't catch up) | Kp too low | Increase Kp by 50% |
| Robot oscillates back and forth, swinging wider and wider until it falls | Kp too high | Reduce Kp by 30% |
| Robot oscillates with constant amplitude (doesn't grow or shrink) | Kp is at the **critical point** | Write this value down! This is `Kp_critical`. Your ideal Kp is **0.5 × Kp_critical** to **0.7 × Kp_critical** |
| Robot stands briefly, vibrates violently, then falls | Kp way too high, or motor direction is wrong | Verify motor direction (Section 2), then reduce Kp by 50% |

### Finding Kp_critical (the gold standard method)

1. Start Kp low (robot falls)
2. Keep doubling Kp until the robot oscillates back and forth without falling — this sustained oscillation is the **critical point**
3. Note this value as `Kp_critical`
4. Set `PID_KP = 0.6 × Kp_critical`
5. The robot should now try to balance but oscillate with declining amplitude (it bounces back and forth but less each time, then eventually falls because there's no damping yet — that's what Kd is for)

> [!IMPORTANT]
> **Don't spend hours on Kp alone.** It's impossible to balance with only Kp — the robot will always oscillate. Your goal is just to find the rough range where the robot "tries" to balance but oscillates. Then move to Kd.

---

## 8. Step 4: Tune Kd (Derivative)

**Kd is what makes the robot actually balance.** It provides damping — the faster the robot is falling, the harder the motors push back. Without Kd, you have a spring with no shock absorber.

### Procedure

1. Keep Kp at the value from Step 3 (roughly 0.6 × Kp_critical)
2. Start with `PID_KD = 0.5`
3. Place robot on the ground and release
4. **Observe:**

| Observation | What's happening | Action |
|:------------|:-----------------|:-------|
| Same oscillation as with Kd=0 | Kd too low, not enough damping | Double Kd |
| Oscillation frequency decreases, amplitude decreases | Kd is starting to work! | Increase Kd by 50% |
| Robot stands! But still sways slowly back and forth | Good Kd, but could use slightly more | Increase Kd by 20% |
| **Robot stands and is stable!** Small corrections, minimal sway | 🎯 **You found it!** | Write this value down |
| Robot stands but vibrates/buzzes rapidly (small, fast oscillation) | Kd too high — amplifying sensor noise | Reduce Kd by 20-30%, or increase `PID_D_FILTER_ALPHA` (see Step 5) |
| Robot is sluggish — when you push it, it corrects very slowly and falls | Kd way too high — over-damped | Reduce Kd by 40% |

### The sweet spot

You're looking for the value where:
- ✅ The robot stands still when undisturbed
- ✅ When you gently push it, it corrects quickly and returns to vertical
- ✅ No visible vibration or buzzing from the motors
- ❌ NOT: buzzing in place, or sluggish/slow to react to pushes

### Iterating Kp and Kd together

After finding a working Kd, you may want to go back and slightly increase Kp (by 10-20%) for a more aggressive response, then re-adjust Kd to match. Two or three iterations of this will converge on the best Kp/Kd pair.

---

## 9. Step 5: Tune PID_D_FILTER_ALPHA

This controls how much the D-term is smoothed. It's a first-order IIR low-pass filter:

```
filteredD = alpha × previous_filteredD + (1 - alpha) × raw_derivative
```

| Alpha | Effect |
|:------|:-------|
| 0.0 | No filtering — raw derivative passes through. Maximum responsiveness, maximum noise. |
| 0.3 | Light filtering. Good if your sensor is clean. |
| 0.5 | Moderate filtering. **Good starting point.** |
| 0.7 | Heavy filtering. Use if motors buzz even at moderate Kd. |
| 0.9 | Very heavy filtering. D-term reacts very slowly. Only use if desperate. |

### When to adjust

- **Motors buzz/vibrate at moderate Kd values** → increase alpha (0.5 → 0.7)
- **Robot is sluggish after you push it** → decrease alpha (0.5 → 0.3). The filter might be slowing down the D-term response.
- **Robot balances but makes rapid micro-corrections** → increase alpha slightly

> [!TIP]
> After changing alpha, you may need to slightly re-tune Kd. Higher alpha effectively reduces the D-term magnitude, so you may need a higher Kd to compensate.

---

## 10. Step 6: Tune MIN_PID_OUTPUT (Dead-Zone)

`MIN_PID_OUTPUT` stops the motors when the PID output magnitude is below this threshold. This prevents the motors from dithering at useless speeds.

| Value | Effect |
|:------|:-------|
| 0 | No dead-zone. Motors always try to step, even at extremely slow rates. |
| 10-20 | Small dead-zone. Motors stop only when nearly balanced. |
| 30-50 | Moderate dead-zone. **Good starting point.** |
| 100+ | Large dead-zone. Robot ignores small tilts — can drift and fall. |

### How to tune

1. Start with `MIN_PID_OUTPUT = 20`
2. If the robot makes annoying clicking/buzzing sounds when nearly balanced → increase to 30-50
3. If the robot slowly drifts to one side and falls without correcting → decrease to 10-15
4. The sweet spot: motors are silent when the robot is upright, but engage immediately when it starts to tilt

---

## 11. Step 7: Tune Ki (Integral) — Maybe

> [!WARNING]
> **Most self-balancing robots do NOT need Ki.** Only add it if you have a specific problem that Kp/Kd cannot solve. Ki is the most dangerous gain — it causes slow oscillations that build up over time (windup), which is very hard to debug.

### When you need Ki

**The only scenario**: The robot balances but consistently leans 2-3° in one direction and never corrects back to vertical. This steady-state error is what Ki fixes — it slowly accumulates the error and pushes the motor harder to correct it.

### When you do NOT need Ki

- Robot oscillates → that's a Kp/Kd problem, not Ki
- Robot falls after a few seconds → that's a Kp/Kd problem
- Robot drifts one way then the other → that's sensor noise, not steady-state error

### If you must use Ki

1. Start with `PID_KI = 0.1` (yes, that small)
2. The integral term's effect builds up over seconds, so observe the robot for 30+ seconds
3. If it still leans, double Ki → 0.2 → 0.4
4. If the robot starts making slow, large oscillations (swaying with a period of 2-5 seconds), Ki is too high. Halve it.
5. **Never set Ki higher than about 2-5% of Kp.** If `Kp = 200`, then `Ki` should be at most ~5-10.

---

## 12. Step 8: Fine-Tune MAX_STEP_INTERVAL

`MAX_STEP_INTERVAL` sets the slowest motor speed. With the current config:
- `MAX_STEP_INTERVAL = 500` → slowest speed = 20 steps/s → 0.375 RPM

At 20 steps/s with 1/16 microstepping, each step is tiny (0.1125°). The motor essentially vibrates in place rather than producing useful rotation. 

### Why it matters

When the PID output is small (robot is nearly balanced), the motor runs at or near this minimum speed. If this speed is too slow, the motor produces no useful torque — it's just buzzing.

| MAX_STEP_INTERVAL | Min speed (steps/s) | Effect |
|:-------------------|:--------------------|:-------|
| 200 | 50 | Motor can go slower. Finer low-speed control, but risk of stalling. |
| 500 | 20 | Default. Very slow minimum. Works with the dead-zone. |
| 1000 | 10 | Extremely slow. Only useful with a very large dead-zone. |

### Recommendation

Keep at 500. The `MIN_PID_OUTPUT` dead-zone handles the "useless speeds" problem more cleanly. Only reduce `MAX_STEP_INTERVAL` if you want finer speed resolution at low speeds.

---

## 13. Step 9: Tune FALLEN_ANGLE_DEG

This is the safety cutoff. When the tilt exceeds this angle, the motors are killed to prevent the robot from wildly spinning its wheels on the ground.

| Value | Effect |
|:------|:-------|
| 30° | Very conservative. Motors cut early. Gives less time for the robot to recover from large disturbances. |
| 45° | Moderate. **Good starting point.** |
| 60° | Aggressive. Gives the robot a lot of time to recover from pushes. Risk: if it's past ~45° it's probably not coming back anyway, and the motors just waste energy. |

### Recommendation

Start with **45°**. After you have good Kp/Kd values and the robot balances well, you can reduce to **35°** to save the motors from unnecessary stress when the robot does fall.

---

## Symptom → Fix Reference Table

Use this table when something goes wrong during tuning:

| Symptom | Most likely cause | Fix |
|:--------|:-----------------|:----|
| Robot falls over immediately, motors barely react | Kp way too low | Double Kp |
| Robot falls over immediately, motors spin at full speed in the WRONG direction | Motor direction inverted | Swap `INVERT_*_MOTOR` flags |
| Robot oscillates back and forth violently, falls | Kp too high | Reduce Kp by 30-40% |
| Robot oscillates with constant amplitude (sustained oscillation) | Kp at critical point, no damping | This is actually perfect — you found Kp_critical. Now add Kd. |
| Robot balances but vibrates/buzzes rapidly | Kd too high, or D-filter too low | Reduce Kd by 20%, or increase `PID_D_FILTER_ALPHA` |
| Robot balances but sways slowly (1-2 sec period) | Kd too low | Increase Kd by 30% |
| Robot balances but slowly drifts one direction | Steady-state error | Add small Ki (0.1-1.0), or check balance angle calibration |
| Robot balances briefly, then oscillations grow and it falls | Ki too high (integral windup) | Reduce Ki by 50%, or set to zero |
| Robot is sluggish — takes too long to recover from a push | Kd too high (over-damped), or D-filter too high | Reduce Kd or reduce `PID_D_FILTER_ALPHA` |
| Motors click/buzz when robot is nearly balanced | MIN_PID_OUTPUT too low | Increase MIN_PID_OUTPUT to 40-60 |
| Robot doesn't correct small tilts, slowly drifts and falls | MIN_PID_OUTPUT too high | Decrease MIN_PID_OUTPUT to 10-20 |
| Robot corrects well for small tilts but can't recover from large pushes | PID_OUTPUT_MAX too low (clipping) | Increase PID_OUTPUT_MAX |
| Robot falls immediately on restart (after being picked up) | Stale FIFO data, or pidReset issue | Already fixed in the code — should not happen |
| Balance angle drifts over time (robot leans more and more) | MPU6050 gyro drift or thermal effect | Increase CALIBRATION_SAMPLES, or re-calibrate periodically |

---

## Recommended Starting Config

Based on your hardware (NEMA17 steppers, 1/16 microstepping, Arduino Nano, MPU6050 DMP at 200Hz):

```cpp
#define PID_KP            80.0f     // start here, adjust per Section 7
#define PID_KI            0.0f      // leave at zero until Section 11
#define PID_KD            1.0f      // start here, adjust per Section 8

#define PID_OUTPUT_MAX    5000.0f   // adjust per Section 6
#define PID_D_FILTER_ALPHA  0.5f   // adjust per Section 9
#define MIN_PID_OUTPUT    30        // adjust per Section 10

#define FALLEN_ANGLE_DEG  45.0f
#define MAX_STEP_INTERVAL 500
#define MIN_STEP_INTERVAL 1

#define CALIBRATION_SAMPLES  100
#define DEBUG_SERIAL      0         // OFF for ground testing!
```

> [!TIP]
> **Keep a tuning log.** After each flash, write down the parameter values and what happened. Something like:
> ```
> Kp=80  Ki=0  Kd=0  → falls slowly, motors react but too weak
> Kp=160 Ki=0  Kd=0  → oscillates, ~2 sec period, falls after 5 swings
> Kp=320 Ki=0  Kd=0  → oscillates faster, ~1 sec period, sustained → Kp_critical ≈ 320
> Kp=200 Ki=0  Kd=1  → oscillation dampened, stands for 3 sec then falls
> Kp=200 Ki=0  Kd=3  → stands! slight sway. push recovery OK
> Kp=200 Ki=0  Kd=5  → stands, very stable, slight buzz → increase D filter
> Kp=200 Ki=0  Kd=5, alpha=0.7 → buzz gone, stable!
> ```
> This log is invaluable when you need to back-track.
