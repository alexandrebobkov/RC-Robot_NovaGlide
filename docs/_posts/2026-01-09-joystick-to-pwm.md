---
layout: default
title: "Mathematics Behind RC Joystic Control"
date: 2026-01-09
categories: robotics
---

# Mathematics Behind RC Joystick Control

## Overview

This post covers the complete mathematical formulation of the joystick-to-motor control pipeline alongside the actual C implementation, showing the direct correspondence between theory and practice.

---

## 1. Input Normalization

### Raw ADC to Normalized Coordinates

**Mathematical Model:**

**Input:**
- $x_{\text{raw}}, y_{\text{raw}} \in [0, 4095]$ (12-bit ADC values)

**Centering:**

$$x_c = x_{\text{raw}} - C_x$$

$$y_c = y_{\text{raw}} - C_y$$

**Normalization:**

$$x_n = \frac{x_c}{R_x}, \quad y_n = \frac{y_c}{R_y}$$

**Clamping:**

$$x_{\text{clamp}} = \text{clamp}(x_n, -1, 1)$$

**Implementation:**

```c
void joystick_hal_update(joystick_hal_t *js, int32_t x_raw, int32_t y_raw)
{
    js->raw_x = x_raw;
    js->raw_y = y_raw;
    
    // STEP 1: Center Correction
    // Implements: x_c = x_raw - C_x
    float dx = (float)x_raw - JS_CENTER_X;
    float dy = (float)y_raw - JS_CENTER_Y;
    
    // STEP 2: Normalization
    // Implements: x_n = x_c / R_x
    float nx = dx / JS_RANGE_X;
    float ny = dy / JS_RANGE_Y;
    
    // STEP 3: Clamping
    // Implements: clamp(x_n, -1, 1)
    if (nx > 1.0f) nx = 1.0f;
    if (nx < -1.0f) nx = -1.0f;
    if (ny > 1.0f) ny = 1.0f;
    if (ny < -1.0f) ny = -1.0f;
    
    // ... continues below
}
```

---

## 2. Deadband Filtering

### Asymmetric Deadband Function

**Mathematical Model:**

$$\text{deadband}(v, d) = \begin{cases}
0 & \text{if } |v| < d \\
v & \text{if } |v| \geq d
\end{cases}$$

**Applied to X and Y:**

$$x = \begin{cases}
0 & \text{if } |x_{\text{clamp}}| < d_x \\
x_{\text{clamp}} & \text{otherwise}
\end{cases}$$

Where $d_x = 0.15, \quad d_y = 0.08$

**Implementation:**

```c
void joystick_hal_update(joystick_hal_t *js, int32_t x_raw, int32_t y_raw)
{
    // ... normalization above ...
    
    // STEP 4: Asymmetric Deadband
    // Implements: deadband(nx, d_x) and deadband(ny, d_y)
    const float deadband_x = 0.15f;  // d_x = 15%
    const float deadband_y = 0.08f;  // d_y = 8%
    
    // X-axis: |nx| < 0.15 → 0
    js->norm_x = (fabsf(nx) < deadband_x) ? 0.0f : nx;
    
    // Y-axis: |ny| < 0.08 → 0
    js->norm_y = (fabsf(ny) < deadband_y) ? 0.0f : ny;
}
```

**Generic Deadband Helper:**

```c
// Implements: deadband(v, d)
static float apply_deadband(float v, float d)
{
    return (fabsf(v) < d) ? 0.0f : v;
}
```

**Graphical Representation:**

```
  Output
    ^
  1 |         ╱
    |        ╱
    |       ╱
 0  |------╱        <- Deadband region [-d, +d]
    |    ╱
 -1 |   ╱
    +----+----+-----> Input
       -1   0   1
```

---

## 3. Non-Linear Response Shaping

### Cubic Exponential Curve

**Mathematical Model:**

$$x_{\text{shaped}} = x^3$$

**Properties:**
- Sign preservation: $\text{sign}(x^3) = \text{sign}(x)$
- Derivative: $\frac{d(x^3)}{dx} = 3x^2$ (sensitivity increases with deflection)

**Response Lookup Table:**

| Input $x$ | Shaped $x^3$ | Sensitivity $3x^2$ |
|-----------|--------------|---------------------|
| 0.0 | 0.000 | 0.000 |
| 0.2 | 0.008 | 0.120 |
| 0.5 | 0.125 | 0.750 |
| 0.7 | 0.343 | 1.470 |
| 1.0 | 1.000 | 3.000 |

**Implementation:**

```c
void joystick_mix(float x, float y, int *pwm_left, int *pwm_right)
{
    // STEP 1: Non-linear Steering Curve
    // Implements: x_shaped = x³
    float x_shaped = x * x * x;
    
    // Alternative curves (commented out in original):
    // Quadratic: float x_shaped = x * fabsf(x);
    // Aggressive cubic: float x_shaped = x * x * x * 1.2f;
    
    // ... continues below
}
```

**Comparison of Response Curves:**

```
Linear:    y = x
Quadratic: y = x²·sign(x)
Cubic:     y = x³         ← YOUR CHOICE (smoothest center)
Quintic:   y = x⁵         (too much dead zone)
```

---

## 4. Differential Drive Mixing

### Basic Mixing Formula

**Mathematical Model:**

**Steering Gain:**

$$x_s = k \cdot x_{\text{shaped}} = 0.9 \cdot x^3$$

**Differential Mixing:**

$$L_0 = y + x_s = y + 0.9x^3$$

$$R_0 = y - x_s = y - 0.9x^3$$

**Physical Meaning:**
- Both motors get $y$ (throttle) component
- Left motor gets $+x_s$ (turns right when positive)
- Right motor gets $-x_s$

**Implementation:**

```c
void joystick_mix(float x, float y, int *pwm_left, int *pwm_right)
{
    // ... shaping above ...
    
    // STEP 2: Steering Gain
    // Implements: x_s = k·x_shaped
    const float k = 0.9f;
    float x_scaled = k * x_shaped;  // k = 0.9
    
    // STEP 3: Differential Mix
    // Implements: L₀ = y + x_s, R₀ = y - x_s
    float L0 = y + k * x_shaped;
    float R0 = y - k * x_shaped;
    
    // ... continues below
}
```

**Effect of Different k Values:**

```c
// k = 0.3:  Very soft steering, wide arcs only
// k = 0.5:  Gentle, limited spin capability
// k = 0.7:  Moderate, good for racing
// k = 0.9:  Aggressive, strong spins ← YOUR CHOICE
// k = 1.2:  Very aggressive, hard to control
```

---

## 5. Differential Limiting

### Arc Limiter (Prevents Over-Aggressive Turning)

**Mathematical Model:**

**Compute Differential:**

$$\Delta = |L_0 - R_0|$$

**Limiting Rule:**

$$\text{If } \Delta > \Delta_{\max}, \text{ apply scaling: } s = \frac{\Delta_{\max}}{\Delta}$$

$$(L_1, R_1) = \begin{cases}
(s \cdot L_0, s \cdot R_0) & \text{if } \Delta > \Delta_{\max} \\
(L_0, R_0) & \text{otherwise}
\end{cases}$$

Where $\Delta_{\max} = 1.7$

**Properties:**
- Preserves ratio: $\frac{L_1}{R_1} = \frac{L_0}{R_0}$
- Reduces magnitude: $|L_1| \leq |L_0|$
- Controls differential: $|L_1 - R_1| \leq \Delta_{\max}$

**Implementation:**

```c
void joystick_mix(float x, float y, int *pwm_left, int *pwm_right)
{
    // ... mixing above ...
    
    // STEP 4: Differential Limiting
    // Implements: Δ = |L₀ - R₀|
    float diff = fabsf(L0 - R0);
    float max_diff = 1.7f;  // Δ_max = 1.7
    
    // If diff > max_diff, scale both proportionally
    // Implements: s = Δ_max / Δ, then L₁ = s·L₀
    if (diff > max_diff) {
        float scale = max_diff / diff;
        L0 *= scale;
        R0 *= scale;
    }
    
    // ... continues below
}
```

**Example:**

```c
// Input: L₀ = 1.2, R₀ = -0.6, diff = 1.8
// Since 1.8 > 1.7:
//   scale = 1.7 / 1.8 = 0.9444
//   L₁ = 1.2 × 0.9444 = 1.133
//   R₁ = -0.6 × 0.9444 = -0.567
//   New diff = 1.7 ✓
```

---

## 6. Final Clamping

**Mathematical Model:**

$$L_{\text{final}} = \text{clamp}(L_1, -1, 1)$$

$$R_{\text{final}} = \text{clamp}(R_1, -1, 1)$$

**Implementation:**

```c
void joystick_mix(float x, float y, int *pwm_left, int *pwm_right)
{
    // ... limiting above ...
    
    // STEP 5: Final Clamping
    // Implements: clamp(L₁, -1, 1)
    if (L0 > 1.0f) L0 = 1.0f;
    if (L0 < -1.0f) L0 = -1.0f;
    if (R0 > 1.0f) R0 = 1.0f;
    if (R0 < -1.0f) R0 = -1.0f;
    
    // ... continues below
}
```

**Helper Function:**

```c
// Generic clamp function
// Implements: clamp(val, min, max)
static float clampf(float val, float min, float max) {
    return (val < min) ? min : (val > max) ? max : val;
}

// Could be used as:
// L0 = clampf(L0, -1.0f, 1.0f);
// R0 = clampf(R0, -1.0f, 1.0f);
```

---

## 7. PWM Scaling

**Mathematical Model:**

$$\text{PWM}_L = \lfloor L_{\text{final}} \times 8190 \rfloor$$

$$\text{PWM}_R = \lfloor R_{\text{final}} \times 8190 \rfloor$$

Where:
- $\lfloor \cdot \rfloor$ = floor function (round to integer)
- Range: $[-8190, +8190]$
- Sign: positive = forward, negative = reverse

**Implementation:**

```c
void joystick_mix(float x, float y, int *pwm_left, int *pwm_right)
{
    // ... clamping above ...
    
    // STEP 6: PWM Scaling
    // Implements: PWM = ⌊L_final × 8190⌋
    *pwm_left  = (int)(L0 * 8190.0f);
    *pwm_right = (int)(R0 * 8190.0f);
}
```

**Why 8190 instead of 8191?**

```c
// Max 13-bit value is 8191 (2^13 - 1)
// Using 8190 provides safety margin for:
// - Rounding errors
// - Motor driver overflow protection
// - Prevents accidental PWM wrap-around
```

---

## Complete Pipeline Example

### Example: Sharp Right Turn

**Input:** $x = 0.9, \quad y = 0.5$

**Step-by-Step Calculation:**

```c
// Given inputs after normalization and deadband:
float x = 0.9f;
float y = 0.5f;

// STEP 1: Cubic shaping
// x_shaped = x³ = (0.9)³ = 0.729
float x_shaped = x * x * x;  // = 0.729

// STEP 2: Steering gain
// x_s = 0.9 × 0.729 = 0.6561
const float k = 0.9f;
float x_s = k * x_shaped;  // = 0.6561

// STEP 3: Differential mix
// L₀ = 0.5 + 0.6561 = 1.1561
// R₀ = 0.5 - 0.6561 = -0.1561
float L0 = y + x_s;  // = 1.1561
float R0 = y - x_s;  // = -0.1561

// STEP 4: Check differential limit
// Δ = |1.1561 - (-0.1561)| = 1.3122
float diff = fabsf(L0 - R0);  // = 1.3122
float max_diff = 1.7f;
// 1.3122 < 1.7 ✓ (no limiting needed)

// STEP 5: Clamping
// L₀ = 1.1561 → clamp to 1.0
// R₀ = -0.1561 → stays -0.1561
if (L0 > 1.0f) L0 = 1.0f;  // L0 becomes 1.0
// R0 unchanged

// STEP 6: PWM scaling
// PWM_L = ⌊1.0 × 8190⌋ = 8190
// PWM_R = ⌊-0.1561 × 8190⌋ = -1278
int pwm_left = (int)(L0 * 8190.0f);   // = 8190
int pwm_right = (int)(R0 * 8190.0f);  // = -1278
```

**Result:** Left motor full forward (8190), right motor reverse (-1278) = tight right turn

---

## 4 Control Scenarios with Code

### Scenario 1: Straight Forward

```c
// INPUT: Full forward, no steering
float x = 0.0f, y = 1.0f;

// CALCULATION:
x_shaped = 0.0f * 0.0f * 0.0f = 0.0f
x_s = 0.9f * 0.0f = 0.0f
L0 = 1.0f + 0.0f = 1.0f
R0 = 1.0f - 0.0f = 1.0f
diff = |1.0 - 1.0| = 0.0 < 1.7  ✓
pwm_left = 8190
pwm_right = 8190

// RESULT: Straight line, full speed
```

### Scenario 2: Gentle Right Arc

```c
// INPUT: Forward with slight right deflection
float x = 0.3f, y = 0.8f;

// CALCULATION:
x_shaped = 0.3f * 0.3f * 0.3f = 0.027f
x_s = 0.9f * 0.027f = 0.0243f
L0 = 0.8f + 0.0243f = 0.8243f
R0 = 0.8f - 0.0243f = 0.7757f
diff = 0.0486 < 1.7  ✓
pwm_left = 6751
pwm_right = 6353

// RESULT: Smooth arc, right motor ~6% slower
```

### Scenario 3: Spin in Place

```c
// INPUT: Maximum rotation, no translation
float x = 1.0f, y = 0.0f;

// CALCULATION:
x_shaped = 1.0f * 1.0f * 1.0f = 1.0f
x_s = 0.9f * 1.0f = 0.9f
L0 = 0.0f + 0.9f = 0.9f
R0 = 0.0f - 0.9f = -0.9f
diff = |0.9 - (-0.9)| = 1.8 > 1.7  ✗ (LIMIT!)

// LIMITING:
scale = 1.7f / 1.8f = 0.9444f
L0 = 0.9f * 0.9444f = 0.85f
R0 = -0.9f * 0.9444f = -0.85f
pwm_left = 6962
pwm_right = -6962

// RESULT: Pivot turn at 85% power
```

### Scenario 4: Backward Right Turn

```c
// INPUT: Reverse with right steering
float x = 0.6f, y = -0.7f;

// CALCULATION:
x_shaped = 0.6f * 0.6f * 0.6f = 0.216f
x_s = 0.9f * 0.216f = 0.1944f
L0 = -0.7f + 0.1944f = -0.5056f
R0 = -0.7f - 0.1944f = -0.8944f
diff = 0.3888 < 1.7  ✓
pwm_left = -4141
pwm_right = -7325

// RESULT: Backing up while turning right
```

---

## Complete Function Implementation

### Full Joystick Mix Function with Annotations

```c
/**
 * Complete implementation with mathematical correspondence
 */
void joystick_mix(float x, float y, int *pwm_left, int *pwm_right)
{
    // ═══════════════════════════════════════════════════════════
    // STAGE 1: Non-linear Shaping
    // Math: x_shaped = x³
    // ═══════════════════════════════════════════════════════════
    float x_shaped = x * x * x;
    
    // ═══════════════════════════════════════════════════════════
    // STAGE 2: Steering Gain
    // Math: x_s = k·x_shaped, where k = 0.9
    // ═══════════════════════════════════════════════════════════
    const float k = 0.9f;
    
    // ═══════════════════════════════════════════════════════════
    // STAGE 3: Differential Mixing
    // Math: L₀ = y + k·x³, R₀ = y - k·x³
    // ═══════════════════════════════════════════════════════════
    float L0 = y + k * x_shaped;
    float R0 = y - k * x_shaped;
    
    // ═══════════════════════════════════════════════════════════
    // STAGE 4: Arc Limiting
    // Math: If |L₀ - R₀| > Δ_max, scale: (L₁,R₁) = (Δ_max/Δ)·(L₀,R₀)
    // ═══════════════════════════════════════════════════════════
    float diff = fabsf(L0 - R0);
    float max_diff = 1.7f;
    
    if (diff > max_diff) {
        float scale = max_diff / diff;
        L0 *= scale;
        R0 *= scale;
    }
    
    // ═══════════════════════════════════════════════════════════
    // STAGE 5: Final Clamping
    // Math: L_final = clamp(L₁, -1, 1)
    // ═══════════════════════════════════════════════════════════
    if (L0 > 1.0f) L0 = 1.0f;
    if (L0 < -1.0f) L0 = -1.0f;
    if (R0 > 1.0f) R0 = 1.0f;
    if (R0 < -1.0f) R0 = -1.0f;
    
    // ═══════════════════════════════════════════════════════════
    // STAGE 6: PWM Scaling
    // Math: PWM = ⌊L_final × 8190⌋
    // ═══════════════════════════════════════════════════════════
    *pwm_left  = (int)(L0 * 8190.0f);
    *pwm_right = (int)(R0 * 8190.0f);
}
```

---

## Parameter Tuning Reference

### Code Constants vs. Mathematical Symbols

| Code Constant | Math Symbol | Value | Location |
|---------------|-------------|-------|----------|
| `deadband_x` | $d_x$ | 0.15 | `joystick_hal_update()` |
| `deadband_y` | $d_y$ | 0.08 | `joystick_hal_update()` |
| `k` | $k$ | 0.9 | `joystick_mix()` |
| `max_diff` | $\Delta_{\max}$ | 1.7 | `joystick_mix()` |
| `8190.0f` | PWM scale | 8190 | `joystick_mix()` |

### Quick Tuning Guide

**To make steering more aggressive:**

```c
// Increase k from 0.9 to 1.1
const float k = 1.1f;  // More aggressive

// Or increase arc limit
float max_diff = 1.9f;  // Allow sharper turns
```

**To make center feel smoother:**

```c
// Use higher exponent (x⁵ instead of x³)
float x_shaped = x * x * x * x * x;  // Quintic

// Or increase X deadband
const float deadband_x = 0.20f;  // Wider dead zone
```

---

## Summary

This code implements the **exact correspondence** between:
- Mathematical theory (equations)
- Practical implementation (C code)
- Real-world behavior (worked examples)

The complete transformation is:

$$(\text{PWM}_L, \text{PWM}_R) = \mathcal{F}(x_{\text{raw}}, y_{\text{raw}})$$

Where $\mathcal{F}$ implements a 6-stage pipeline optimized for intuitive robot control.
