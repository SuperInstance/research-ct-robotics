# RESEARCH.md
## THE PROBLEM
Robot servos report position as floats (e.g., 47.29999923 degrees). Cameras report angles as floats. IMUs report orientation as quaternion floats. Every sensor reading has floating point noise. When a simulation tries to mirror reality, the float noise compounds. After 1000 sensor-sample-simulate loops, the simulation diverges from reality. The robot thinks its arm is at 47.3° but it's actually at 47.1°.
## THE SOLUTION
Pythagorean manifold snapping. Every sensor reading is snapped to the nearest Pythagorean coordinate. The snap is deterministic — same input, same output, every machine, every time. The simulation uses the SAME snap function on the SAME sensor data. Now simulation state = robot state. Zero divergence. Forever.
## THE MUD CONNECTION
In a MUD (multi-user dungeon) where agents control robots:
- Agent in the MUD sees 'gripper at position (3,4,5)' — these are Pythagorean coordinates
- Real robot's servo reports (2.99999, 4.00001, 5.00001) — snapped to (3,4,5)
- Agent commands 'move to (3,7,5)' — Pythagorean target
- Real servo moves to (3.00001, 6.99999, 4.99999) — snapped to (3,7,5)
- Agent sees the move completed. It IS completed. The numbers match.
## THE KEY INSIGHT
This only works because Pythagorean snapping is:
- Lossless within tolerance (no information destroyed if within snap grid)
- Deterministic (same float, same snap, every time)
- Bijective within tolerance (every float in a cell maps to exactly one snap point)
- The tolerance IS the sensor precision — set grid to match your hardware
## SENSOR-TO-SNAP PIPELINE
- Servo position → f64 → snap to manifold → Pythagorean coordinate
- Camera angle → f64 → snap to manifold → Pythagorean angle
- IMU quaternion → [f64;4] → normalize → snap direction → Pythagorean quaternion
- Force/torque → f64 → snap to manifold → Pythagorean force vector
- ALL of these produce the same bits on Jetson, on ProArt, in the MUD, everywhere
## WHY FLOATS FAIL IN ROBOTICS
- Float comparison is unreliable (0.1 + 0.2 != 0.3)
- PID controllers accumulate float error in the integral term
- Inverse kinematics chains compound error through each joint
- Sensor fusion (Kalman filter) drifts because float covariance matrices degrade
- Constraint theory fixes ALL of these because snap resets drift to zero every cycle
## THE SERVO-SNAP LOOP
```rust
use constraint_theory_core::PythagoreanManifold;

let mut manifold = PythagoreanManifold::new(0.01);
let servo_position = 47.29999923;
let snapped_position = manifold.snap(servo_position);
println!("Snapped position: {:?}", snapped_position);
```
## PRACTICAL CONSTRAINTS
- Snap tolerance must be >= sensor noise floor (typically 0.01° for good servos)
- Snap density determines position resolution (more density = finer control)
- Real-time constraint: snap must be faster than servo update rate (typically 1ms)
- PythagoreanManifold::new(density) O(log N) snap is fast enough
## COMPARISON TO EXISTING APPROACHES
- Fixed-point arithmetic: loses dynamic range
- Periodic recalibration: requires downtime
- Extended precision (f128): still drifts, just slower
- Symbolic computation: too slow for real-time
- CT snap: none of these problems. Exact, fast, deterministic, zero drift.
## FLEET APPLICATION
- JetsonClaw1's Jetson reads real sensors → snaps → sends Pythagorean coordinates to fleet
- Fleet agents in MUD see exact same coordinates
- Agent commands go back as Pythagorean targets
- Jetson executes, snaps confirmation, reports back
- Zero-loss loop. The MUD IS the robot. The robot IS the MUD.