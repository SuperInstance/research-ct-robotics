# Research: Constraint Theory for Robotics

> **Eliminating sensor-simulation divergence with Pythagorean manifold snapping**

## Overview

Robot servos, cameras, and IMUs all report floating-point readings. After thousands of sensor-sample-simulate loops, float noise compounds and the simulation diverges from reality—the robot thinks its arm is at 47.3° but it's actually at 47.1°. This research demonstrates that Pythagorean manifold snapping eliminates this divergence entirely. Every sensor reading is snapped to the nearest Pythagorean coordinate deterministically, producing identical bits on a Jetson, a workstation, or inside a MUD (multi-user dungeon) agent. The simulation uses the same snap function on the same sensor data, guaranteeing simulation state equals robot state—zero divergence, forever.

## Architecture

```
   Physical Robot                    Simulation / MUD
 ┌───────────────┐              ┌─────────────────────┐
 │ Servo: 47.30° │              │ Agent sees: (3,4,5) │
 │ Cam:   12.45° │              │ Gripper pos snapped │
 │ IMU:   quat   │              │                     │
 └───────┬───────┘              └──────────┬──────────┘
         │                                  │
         ▼                                  │
 ┌───────────────┐                          │
 │  Pythagorean  │◀──── Deterministic ──────┘
 │  Snap         │      (same bits everywhere)
 │  ┌─────────┐  │
 │  │ f64→PT │  │
 │  │ O(logN)│  │
 │  └─────────┘  │
 └───────┬───────┘
         │
         ▼
 ┌───────────────┐     ┌─────────────────────┐
 │ Snapped State │────▶│ Fleet Communication │
 │ (exact,       │     │ (Pythagorean coords)│
 │  deterministic│     │                     │
 └───────────────┘     └─────────────────────┘
```

### Sensor-to-Snap Pipeline

```
  Servo position → f64  → snap → Pythagorean coordinate
  Camera angle   → f64  → snap → Pythagorean angle
  IMU quaternion → [f64;4] → normalize → snap → Pythagorean quaternion
  Force/torque   → f64  → snap → Pythagorean force vector

  ALL produce identical bits on Jetson, ProArt, and in the MUD
```

## Features

- **Lossless within tolerance** — No information destroyed if within snap grid
- **Deterministic** — Same float, same snap, every machine, every time
- **Bijective within tolerance** — Every float in a cell maps to exactly one snap point
- **Zero drift** — PID integral error, inverse kinematics chain error, and Kalman filter drift all reset to zero every cycle
- **Real-time capable** — O(log N) snap faster than typical servo update rates (1ms)
- **MUD-robot bridge** — Simulation state = robot state; the MUD IS the robot

## Why Floats Fail in Robotics

| Problem | Float Behavior | CT Snap Fix |
|---------|---------------|-------------|
| Float comparison | `0.1 + 0.2 != 0.3` | Snapped values are exact integers |
| PID accumulation | Integral term drifts | Snap resets drift to zero |
| Inverse kinematics | Error compounds per joint | Each joint snap is independent |
| Kalman covariance | Degenerates over time | Covariance stays positive-definite |

## Quick Start

This is a research repository. The snap concept is implemented in `constraint_theory_core::PythagoreanManifold`:

```rust
use constraint_theory_core::PythagoreanManifold;

let mut manifold = PythagoreanManifold::new(0.01); // 0.01° tolerance
let servo_position = 47.29999923;
let snapped = manifold.snap(servo_position);
// Result: deterministic Pythagorean coordinate
```

```bash
# Clone the research repo
git clone <repo-url> research-ct-robotics
cd research-ct-robotics

cat RESEARCH.md  # Full research writeup with examples
```

### Practical Constraints

- Snap tolerance must be >= sensor noise floor (typically 0.01° for good servos)
- Snap density determines position resolution
- Snap must be faster than servo update rate (O(log N) is sufficient)

## Integration

| Platform | Role | Snap Point |
|----------|------|------------|
| **JetsonClaw1** | Reads real sensors, snaps, sends to fleet | Edge device |
| **MUD agents** | See exact same Pythagorean coordinates | Simulation |
| **Fleet network** | Commands go back as Pythagorean targets | Communication |
| **GPU pipelines** | Exact uncertainty propagation | Compute |

### Comparison to Existing Approaches

- **Fixed-point arithmetic** — Loses dynamic range → CT snap retains it
- **Periodic recalibration** — Requires downtime → CT snap is continuous
- **Extended precision (f128)** — Still drifts → CT snap eliminates drift
- **Symbolic computation** — Too slow for real-time → CT snap is O(log N)

---

<img src="callsign1.jpg" width="128" alt="callsign">
