# Combat Drone Simulator — Processing MPUTeapot Style
## Setup Instructions

### Requirements
- Processing 4.x  →  https://processing.org/download
- No extra libraries needed (uses built-in `Serial`)

---

### Quick Start (Simulation Mode — No Arduino Needed)

1. Open `CombatDroneSimulator.pde` in Processing
2. Make sure this line is set to `true`:
   ```java
   boolean SIMULATION_MODE = true;
   ```
3. Press ▶ Run

**Keyboard controls:**

| Key | Action |
|-----|--------|
| W / S | Pitch forward / backward |
| A / D | Roll left / right |
| Q / E | Yaw left / right |
| ↑ / ↓ | Throttle up / down |
| R | Reset physics |
| L | Toggle 3D labels |
| P | Screenshot |
| + / - | Tune Kp gain live |
| Drag | Orbit camera |
| Scroll | Zoom |

---

### With Real MPU6050 Arduino

1. Flash `mpu6050_drone.ino` to your Arduino Uno/Nano
2. Open `CombatDroneSimulator.pde`
3. Change these lines:
   ```java
   final String COM_PORT    = "COM3";   // your port, e.g. /dev/ttyUSB0
   boolean SIMULATION_MODE = false;
   ```
4. Press ▶ Run

**Serial format expected (from mpu6050_drone.ino):**
```
Pitch:8.3° Roll:-2.1° Gz:0.5°/s
```

---

### Physics Equations (from blackboard)

| Equation | Role |
|----------|------|
| `θ = α·(θ + Gx·dt) + (1−α)·θ_acc` | MPU6050 complementary filter (α=0.96) |
| `τ = −Kp·e − Kd·ė − Ki·∫e dt` | PID attitude torques |
| `d = (Mix)⁻¹·U` | Motor mixing → FL/FR/RL/RR PWM |
| `J·ω̇ = τ_mix − ω×(Jω)` | Euler rotation with gyroscopic coupling |
| `mP̈ = −cd·Ṗ + f` | Translational dynamics with drag |

**Inertia matrix (J):**
- Ixx (roll)  = 0.0082 kg·m²
- Iyy (pitch) = 0.0085 kg·m²
- Izz (yaw)   = 0.0148 kg·m²

**Mixing matrix:**
```
       T    τφ   τθ   τψ
FL  [ +1   -1   +1   +1 ]
FR  [ +1   +1   +1   -1 ]
RL  [ +1   -1   -1   -1 ]
RR  [ +1   +1   -1   +1 ]
```

---

### What You See

- **3D drone model** — body, arms, motors, propellers, landing gear, camera, weapon arm, MPU6050 chip with I²C wire
- **Spinning propellers** — speed proportional to computed PWM
- **Physics-driven attitude** — drone tilts/rotates according to Euler equations
- **Altitude** — drone rises/falls based on thrust vs gravity
- **Attitude indicator** — artificial horizon (roll + pitch)
- **Altitude bar** — right side of viewport
- **Mini plots** — pitch history, altitude history, mean PWM
- **Motor PWM bars** — FL/FR/RL/RR in real time
- **HUD** — command, mode, altitude readout
