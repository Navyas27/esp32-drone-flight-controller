# ESP32 Quadcopter Flight Controller Firmware Architecture

## Overview

The flight controller firmware is designed for high-rate, low-latency multirotor stabilization on the dual-core **ESP32** microcontroller. It implements a **cascaded dual-loop PID architecture** running at a deterministic **500 Hz (2.0 ms period)**, paired with **ESP-NOW wireless control** for fast telemetry and low-jitter pilot input.

---

## Control Architecture: Cascaded PID

```text
 Pilot Input (Glove)
        │
        ▼
┌──────────────┐    Target Angles    ┌─────────────────┐   Target Rates    ┌────────────────┐   PWM Pulse
│ Angle Mode   │ ──────────────────► │  Outer Loop:    │ ────────────────► │  Inner Loop:   │ ─────────────► Motor Mixer
│ Command Map  │                     │  Angle P-Loop   │                   │  Rate PID Loop │
└──────────────┘                     └─────────────────┘                   └────────────────┘
                                              ▲                                     ▲
                                              │                                     │
                                    Estimated Attitude                       Measured Gyro Rate
                                     (Complementary Filter)                   (MPU6050 Gyro)
```

### 1. Outer Loop (Angle P-Controller)
- Runs attitude stabilization comparing requested tilt angle against estimated angle.
- Produces a target angular velocity demand (deg/s) for the inner loop.
- Formula:
  $$\text{Rate}_{\text{target}} = K_{p,\text{angle}} \times (\text{Angle}_{\text{target}} - \text{Angle}_{\text{actual}})$$

### 2. Inner Loop (Rate PID Controller)
- Directly stabilizes high-frequency rotational dynamics.
- Compares target angular velocity with calibrated gyro readings:
  $$\text{Error}_{\text{rate}} = \text{Rate}_{\text{target}} - \text{Gyro}_{\text{dps}}$$
  $$P = K_p \times \text{Error}_{\text{rate}}$$
  $$I = I_{\text{prev}} + K_i \times \text{Error}_{\text{rate}} \times \Delta t \quad (\text{clamped to } \pm \text{Limit}_I)$$
  $$D = K_d \times \frac{\text{Error}_{\text{rate}} - \text{Error}_{\text{prev}}}{\Delta t}$$
  $$\text{Output}_{\text{PID}} = P + I + D$$

### 3. Attitude Estimation (Complementary Filter)
At 500 Hz, accelerometer and gyroscope data are fused:
$$\theta_t = \alpha \times (\theta_{t-1} + \omega \cdot \Delta t) + (1 - \alpha) \times \theta_{\text{accel}}$$
Where:
- $\alpha = 0.985$ (98.5% Gyro integration + 1.5% Accel gravity reference)
- $\Delta t = 0.002\,\text{s}$ (500 Hz loop)

---

## Quad-X Motor Mixing Equations

```text
       Front
    M4 (FL)   M1 (FR)
     [CW]       [CCW]
        \       /
         \  X  /
          \   /
         /  X  \
        /       \
     [CCW]     [CW]
    M3 (RL)   M2 (RR)
        Rear
```

$$\begin{aligned}
M_1\ (\text{Front-Right, CCW}) &= \text{Throttle} - \text{Pitch} - \text{Roll} + \text{Yaw} \\
M_2\ (\text{Rear-Right, CW})   &= \text{Throttle} + \text{Pitch} - \text{Roll} - \text{Yaw} \\
M_3\ (\text{Rear-Left, CCW})  &= \text{Throttle} + \text{Pitch} + \text{Roll} + \text{Yaw} \\
M_4\ (\text{Front-Left, CW})   &= \text{Throttle} - \text{Pitch} + \text{Roll} - \text{Yaw}
\end{aligned}$$

Each calculated motor signal is constrained to $[1000, 2000]\,\mu\text{s}$. If the drone is disarmed or killed, all channels are immediately set to $1000\,\mu\text{s}$.

---

## PID Default Parameters

```cpp
// Outer Loop: Angle P-Controller
const float ANGLE_KP_ROLL     = 4.0f;
const float ANGLE_KP_PITCH    = 4.0f;

// Inner Loop: Rate PID Controllers
// Roll Rate PID
const float RATE_KP_ROLL      = 0.65f;
const float RATE_KI_ROLL      = 0.25f;
const float RATE_KD_ROLL      = 0.020f;

// Pitch Rate PID
const float RATE_KP_PITCH     = 0.65f;
const float RATE_KI_PITCH     = 0.25f;
const float RATE_KD_PITCH     = 0.020f;

// Yaw Rate PID (Direct rate loop)
const float RATE_KP_YAW       = 2.00f;
const float RATE_KI_YAW       = 0.40f;
const float RATE_KD_YAW       = 0.000f;

// Anti-Windup Clamps (microseconds)
const float PID_I_LIMIT_ROLL  = 120.0f;
const float PID_I_LIMIT_PITCH = 120.0f;
const float PID_I_LIMIT_YAW   = 100.0f;

// Max Total PID Output Clamp per Axis (microseconds)
const float PID_OUT_LIMIT     = 350.0f;
```

---

## Safety Systems & Interlocks

1. **Boot Disarm**: ESC outputs are locked to $1000\,\mu\text{s}$ prior to any sensor or network setup.
2. **2-Second Low-Throttle Arming**: To arm, pilot must hold throttle $\le 1050\,\mu\text{s}$ for at least 2 full seconds with valid wireless connection.
3. **Packet Timeout Failsafe**: If no valid ESP-NOW packet is received within 500 ms, the flight controller disarms immediately and spins down all motors to $1000\,\mu\text{s}$.
4. **Emergency Serial Kill Switch**: Sending `k` or `K` over the serial console ($115200$ baud) enters a locked `STATE_KILLED` mode immediately.
5. **Integral Anti-Windup**: Integrators are cleared whenever throttle is at idle or drone is disarmed, preventing throttle surge upon liftoff.
