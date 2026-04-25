# Project Description: Path-Centric RoboRacer Autonomy Stack

## 1. Objective

The goal of this refactor is to turn the original RoboRacer autonomy package from a **reactive heading controller** into a **trajectory-tracking racing stack** that is better suited to closed-loop time-attack racing.

The original stack could keep the vehicle alive, but it had structural limits:

- it planned a scalar heading instead of a path
- it used follow-the-gap too often as the main planner
- it used dead reckoning as the main pose source
- it mixed perception and planning in a way that amplified sensor jitter
- it sent steering commands without a strong inner steering loop

The upgraded stack changes the architecture to:

`wheel odometry prior -> LiDAR boundary extraction -> centerline / raceline trajectory -> adaptive pure pursuit -> FTG fallback`

It also adds a clean integration point for map-based localization and offline raceline loading.

## 2. Design goals

The upgrade was designed around five goals.

### Goal 1: Make the stack path-based
The most important structural change is that the planner now outputs a **trajectory** rather than a heading. This allows the controller to track a path consistently instead of reacting to momentary scan geometry.

### Goal 2: Keep the runtime stack competition-legal
The package still directly consumes only:

- LiDAR
- front camera
- IMU
- wheel encoders
- steering feedback

An external pose estimate can be injected, but only through a custom topic that should itself be produced from legal sensors.

### Goal 3: Promote LiDAR to the primary racing sensor
The LiDAR module now produces:

- left boundary
- right boundary
- centerline
- width estimate
- blocked-path and clearance metrics

instead of only a gap target.

### Goal 4: Reserve FTG for emergencies
Follow-the-gap is still included, but only as a fallback when:

- the track model is weak
- the path is blocked
- no valid trajectory is available

### Goal 5: Make future upgrades easier
The new data model and planner/controller interfaces make it much easier to add:

- scan-based localization
- map-based raceline tracking
- kinematic MPC
- MPCC
- learned camera perception

## 3. Module architecture

## 3.1 State estimation: `state_estimator.py`

### What it does
This module now acts as a **wheel-odometry prior**, not the final localization layer.

### Algorithm
- steering feedback is converted from normalized actuator feedback to steering angle
- left and right wheel speeds are estimated from encoder angle deltas
- IMU yaw and yaw-rate are fused with bicycle-model prediction
- the pose is propagated with wheelbase-aware kinematics
- a confidence score is produced and decayed over time

### Why this is better
The previous estimator behaved more like a light dead-reckoner without a proper bicycle-model yaw update. The new version is a better prior for both:

- local trajectory tracking
- future scan-based localization

### Critical implementation detail
The simulator steering feedback topic is normalized, not directly an angle in radians. Interpreting it as radians can materially degrade steering prediction.

## 3.2 Perception: `perception.py`

### 3.2.1 LiDAR track extraction

#### What it does
Converts the scan into a local track model.

#### Algorithm
1. clip and smooth the scan
2. convert each range sample into local XY
3. split points into left and right sets
4. bin points by forward distance
5. pick the closest border point in each bin
6. smooth the border traces
7. form a centerline from both borders or from one border plus a width estimate
8. compute confidence, width statistics, heading hint, curvature hint, TTC, and blockage

#### Why this is better
A racing controller wants a **stable corridor**, not an instantaneous gap angle. Binned boundaries and a smoothed centerline are much more stable inputs to a controller.

### 3.2.2 Gap fallback

#### What it does
Provides emergency steering when the normal path source becomes unreliable.

#### Algorithm
- classic follow-the-gap bubble masking
- largest-gap search
- target selection with continuity penalty

#### Why it remains in the stack
FTG is still valuable for:
- obstacle avoidance
- recovery behavior
- degraded perception conditions

It is simply no longer the main planner.

### 3.2.3 Camera auxiliary perception

#### What it does
Extracts weak boundary cues from the forward camera.

#### Algorithm
- ROI crop
- grayscale + Gaussian blur
- Canny edges
- probabilistic Hough transform
- left/right line fitting
- estimate center offset and heading error

#### Role in the system
The camera is intentionally kept **secondary**. It is used to bias fallback behavior and support LiDAR when track confidence drops.

## 3.3 Planning: `planning.py`

### What it does
Produces a local trajectory suitable for path tracking.

### Planning hierarchy
The planner supports three path sources in priority order.

#### Source 1: global raceline
If a raceline CSV is provided and pose quality is good, the planner:
- finds the nearest raceline station
- extracts a local horizon window
- validates it against the local LiDAR corridor

#### Source 2: local centerline
If no valid raceline is available, the planner converts the LiDAR centerline into world coordinates and uses that as the local path.

#### Source 3: gap fallback path
If neither the global nor local path is usable, the planner generates a short constant-curvature recovery trajectory from FTG.

### Speed profile generation
For any path generated from points, the planner computes:

1. arc length
2. heading
3. curvature
4. curvature-limited speed
5. backward braking pass
6. forward acceleration pass

This is much better than deriving speed only from the instantaneous scan.

## 3.4 Mission logic: `mission.py`

### What it does
Runs the behavior state machine.

### Modes
- `BOOTSTRAP`
- `LOCALIZE`
- `RACE`
- `AVOID`
- `RECOVERY`
- `SAFETY_BRAKE`

### Main logic
- stale sensor debounce
- TTC and clearance safety brake
- hysteretic `AVOID` entry and exit
- `LOCALIZE` when pose confidence is weak but tracking is still possible
- `RACE` when the system is healthy

### Why this is better
The old stack switched behavior mainly from gap angle and blockage. The new version switches behavior based on **path validity and safety state**, which is much more appropriate for a racing stack.

## 3.5 Control: `control.py`

### What it does
Tracks the trajectory produced by the planner.

### Algorithm
1. find the nearest point on the current trajectory
2. select a goal point at the dynamic lookahead distance
3. transform that goal into the vehicle frame
4. compute pure-pursuit curvature
5. blend it with the reference path curvature
6. convert to desired steering angle
7. apply steering feedforward and steering feedback
8. apply yaw-rate damping
9. rate-limit steering
10. track target speed with feedforward + PID

### Why this is better
The controller now uses:
- a real trajectory
- dynamic lookahead
- steering feedback
- rate-limited steering
- a proper inner steering correction term

instead of only commanding curvature from a reactive heading.

## 3.6 ROS 2 integration: `autonomy_node.py`

### What it does
Connects the upgraded modules to the simulator topics and publishes commands and debug outputs.

### Published interfaces
- `/roboracer_autonomy/pose_estimate`
- `/roboracer_autonomy/wheel_odom`
- `/roboracer_autonomy/mission_mode`
- `/roboracer_autonomy/target_speed`
- `/roboracer_autonomy/reference_source`

### Key runtime feature
An optional `external_pose_topic` can be supplied for legal map-based localization without changing the rest of the stack.

## 4. Why this stack should perform better

The expected gains come from structural improvements, not from cosmetic tuning.

### 4.1 Less steering jitter
Boundary fitting and centerline smoothing reduce scan-to-scan path jumps.

### 4.2 Better cornering behavior
The controller follows a path with a speed profile instead of reacting to a single heading error.

### 4.3 Less mode chatter
Mission logic now uses hysteresis and path validity instead of reacting directly to ordinary corner geometry.

### 4.4 Better use of steering bandwidth
The controller uses steering feedback and a faster rate limit closer to the actuator capability.

### 4.5 Better upgrade path
The system is now naturally compatible with:
- SLAM and map localization
- offline raceline generation
- MPC / MPCC

## 5. What is already implemented vs. what is left as an integration hook

### Already implemented
- wheel-odometry prior
- LiDAR boundary extraction
- centerline generation
- local trajectory planning
- optional raceline CSV loading
- adaptive pure pursuit tracking
- FTG fallback
- path-validity mission logic
- optional external pose hook
- module-level documentation

### Left as an integration hook
- actual SLAM / AMCL / scan matcher node
- offline raceline optimization tool
- kinematic MPC / MPCC controller
- learned camera segmentation or obstacle detector

These were not forced into this code package because they normally depend on additional ROS packages, optimization solvers, or datasets.

## 6. Tuning philosophy

Tuning should be done in this order:

1. steering feedback interpretation and odometry prior
2. LiDAR boundary extraction
3. pure pursuit lookahead
4. speed profile limits
5. safety thresholds
6. steering bandwidth
7. external localization
8. MPC, if desired

Trying to tune the controller before the centerline is stable or before steering feedback is interpreted correctly will waste a lot of time.

## 7. High-ceiling future plan

The next best competitive configuration is:

- map build with SLAM
- scan-based localization
- saved raceline CSV
- this upgraded planner/controller stack
- later swap pure pursuit for kinematic MPC or MPCC

That keeps the current code useful while moving toward a genuinely high-performance racing system.
