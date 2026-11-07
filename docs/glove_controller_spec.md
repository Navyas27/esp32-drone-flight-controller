# Wireless Glove Controller & ESP-NOW Protocol

## 1. Overview

The quadcopter utilizes **ESP-NOW**, a connectionless Wi-Fi protocol developed by Espressif that allows direct frame transfer with sub-millisecond latencies and no external router requirement.

- **Drone MAC Address (STA)**: `78:1C:3C:CB:CF:A8`
- **Wi-Fi Channel**: Channel 1 (2.4 GHz)
- **Transmission Rate**: 50 Hz – 100 Hz (every 10 ms to 20 ms)
- **Failsafe Timeout**: 500 ms (auto-disarm upon packet loss)

---

## 2. Command Packet Structure

Both transmitter and receiver must share an identical packed C-struct:

```cpp
typedef struct __attribute__((packed)) {
  int16_t roll;      // Roll tilt:    -500 .. +500 -> mapped to -30° .. +30°
  int16_t pitch;     // Pitch tilt:   -500 .. +500 -> mapped to -30° .. +30°
  int16_t yaw_rate;  // Yaw rate:     -500 .. +500 -> mapped to -150°/s .. +150°/s
  int16_t throttle;  // Throttle PWM: 1000 .. 2000 µs
} GloveCommand;
```

---

## 3. Glove Gesture Mapping

| Hand Motion | Glove IMU / Flex Sensor Input | Drone Command | Action |
| :--- | :--- | :--- | :--- |
| **Tilt Hand Forward** | Pitch down ($\theta < 0$) | Pitch $< 0$ | Drone pitches forward (flies forward) |
| **Tilt Hand Backward** | Pitch up ($\theta > 0$) | Pitch $> 0$ | Drone pitches backward (flies backward)|
| **Roll Hand Right** | Roll right ($\phi > 0$) | Roll $> 0$ | Drone banks right |
| **Roll Hand Left** | Roll left ($\phi < 0$) | Roll $< 0$ | Drone banks left |
| **Twist Wrist** | Gyro Yaw ($\omega_z$) | Yaw Rate $\ne 0$ | Drone pivots along yaw axis |
| **Flex Sensor / Thumb Throttle**| Flex bend resistance | Throttle (1000–2000 µs)| Vertical climb / descent rate |

---

## 4. Pairing & Handshake Sequence

1. **Boot**: The drone starts up in `STATE_DISARMED`.
2. **Listen**: ESP-NOW callback registers incoming packets on channel 1.
3. **Arming Interlock**:
   - Pilot holds throttle $\le 1050\,\mu\text{s}$ continuously for $2000\,\text{ms}$.
   - Drone state switches from `STATE_DISARMED` to `STATE_ARMED`.
4. **Flight Control**: Active commands are processed by the cascaded PID loop at 500 Hz.
5. **Loss of Link Failsafe**:
   - If no packet is received for $500\,\text{ms}$, state switches to `STATE_DISARMED`.
   - Motors instantly drop to $1000\,\mu\text{s}$ (idle cutoff).
