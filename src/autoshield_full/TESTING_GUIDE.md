# AutoShield Full System Testing Guide

Complete test cases for verifying the autonomous vehicle safety system.

open this for more reference 
https://docs.google.com/document/d/1r7jDbF8tb5aT5U_DYjy2ucFfG8mwpd4THK4drNA-rtk/edit?tab=t.0#heading=h.5gwd6osiprk7

---

## 🎯 Test Categories

1. [Sensor Fusion Tests](#1-sensor-fusion-tests)
2. [High-Level Decision Tests](#2-high-level-decision-tests)
3. [Controller Tests](#3-controller-tests)
4. [Integration Tests](#4-integration-tests)
5. [Safety-Critical Tests](#5-safety-critical-tests)
6. [Edge Cases & Failure Modes](#6-edge-cases--failure-modes)

---

## 1. Sensor Fusion Tests

### Test 1.1: Perfect Match (Same Person from Both Sensors)
**Objective:** Verify fusion of matched Lidar-Camera detections

**Setup:**
```bash
ros2 run autoshield_full lidar_camera_fusion

also run sunny's code
```

**Input:**
```bash
# Lidar sees person at 14m, 85°
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [14, 85]"

# Camera sees same person at 15m, 87° (within 2m threshold)
ros2 topic pub /rgbd_pedestrian_position std_msgs/msg/Int32MultiArray "data: [15, 87]"
```

**Expected Output:**
```bash
ros2 topic echo /fusion_pedestrian_position
# Should output: Fused detection ~[14, 86]
# Distance: 0.8*14 + 0.2*15 = 14.2 → 14m
# Direction: 0.3*85 + 0.7*87 = 86.4 → 86°
```

**Pass Criteria:**
- ✅ Fused distance between 14-15m
- ✅ Fused direction between 85-87°
- ✅ Log shows "matched" detection

---

### Test 1.2: Lidar-Only Detection (Camera Missed)
**Objective:** Verify Lidar-only detections are kept

**Input:**
```bash
# Lidar sees person
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [20, 120]"

# Camera sees nothing
ros2 topic pub /rgbd_pedestrian_position std_msgs/msg/Int32MultiArray "data: []"
```

**Expected Output:**
```bash
# Should output: [20, 120] (Lidar-only kept)
```

**Pass Criteria:**
- ✅ Detection published with original Lidar values
- ✅ Log shows "lidar-only" detection

---

### Test 1.3: Camera-Only Detection (Lidar Missed)
**Objective:** Verify Camera-only detections trigger warning

**Input:**
```bash
# Lidar sees nothing
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: []"

# Camera sees person
ros2 topic pub /rgbd_pedestrian_position std_msgs/msg/Int32MultiArray "data: [8, 45]"
```

**Expected Output:**
```bash
# Should output: [8, 45] (Camera-only kept)
```

**Pass Criteria:**
- ✅ Detection published with original Camera values
- ✅ WARNING logged about Lidar missing reflection
- ✅ Log shows "camera-only" detection

---

### Test 1.4: Multiple Detections (2 People)
**Objective:** Verify handling of multiple pedestrians

**Input:**
```bash
# Lidar sees 2 people
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [14, 85, 20, 120]"

# Camera sees 2 people (one matches, one different)
ros2 topic pub /rgbd_pedestrian_position std_msgs/msg/Int32MultiArray "data: [15, 87, 8, 45]"
```

**Expected Output:**
```bash
# Should output: 3 detections
# 1. Fused (14,85)+(15,87) → ~[14, 86]
# 2. Lidar-only [20, 120]
# 3. Camera-only [8, 45]
```

**Pass Criteria:**
- ✅ 3 detections published
- ✅ 1 matched pair
- ✅ 1 lidar-only
- ✅ 1 camera-only

---

### Test 1.5: No Detections (Empty Arrays)
**Objective:** Verify graceful handling of no pedestrians

**Input:**
```bash
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: []"
ros2 topic pub /rgbd_pedestrian_position std_msgs/msg/Int32MultiArray "data: []"
```

**Expected Output:**
```bash
# Should output: empty array []
```

**Pass Criteria:**
- ✅ Empty array published (no crash)
- ✅ Log shows "No detections from either sensor"

---

### Test 1.6: Angle Wrapping (0°/360° Boundary)
**Objective:** Verify correct angle averaging across boundary

**Input:**
```bash
# Lidar at 350° (near 360°)
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 350]"

# Camera at 10° (near 0°)
ros2 topic pub /rgbd_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 10]"
```

**Expected Output:**
```bash
# Should average to ~0°, NOT 180°
# 0.3*350 + 0.7*370 = 364 % 360 = 4°
```

**Pass Criteria:**
- ✅ Fused angle close to 0-10° (NOT ~180°)
- ✅ No angle wrapping errors

---

### Test 1.7: Matching Threshold Test
**Objective:** Verify 2.0m matching threshold

**Input:**
```bash
# Person 1: Lidar at (10m, 0°) → (10, 0) in XY
# Person 2: Camera at (13m, 0°) → (13, 0) in XY
# Distance = 3m > 2m threshold → Should NOT match
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 0]"
ros2 topic pub /rgbd_pedestrian_position std_msgs/msg/Int32MultiArray "data: [13, 0]"
```

**Expected Output:**
```bash
# Should output: 2 separate detections (no match)
# [10, 0] and [13, 0]
```

**Pass Criteria:**
- ✅ 2 separate detections published
- ✅ No matching logged

---

## 2. High-Level Decision Tests

### Test 2.1: CRUISE (No Pedestrian)
**Objective:** Verify normal driving state

**Setup:**
```bash
ros2 run autoshield_full high_level_command
```

**Input:**
```bash
# No pedestrian detected
ros2 topic pub /fusion_pedestrian_position std_msgs/msg/Int32MultiArray "data: []"
ros2 topic pub /pedestrian_sign_present std_msgs/msg/Bool "data: false"
ros2 topic pub /pedestrian_motion geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}}"
ros2 topic pub /pedestrian_ttc std_msgs/msg/Float64 "data: 100.0"
```

**Expected Output:**
```bash
ros2 topic echo /safety_decision
# data: "CRUISE"
```

**Pass Criteria:**
- ✅ State: CRUISE
- ✅ Patience timer reset

---

### Test 2.2: STOP_YIELD (Critical TTC)
**Objective:** Verify collision avoidance override

**Input:**
```bash
# Pedestrian detected with critical TTC
ros2 topic pub /fusion_pedestrian_position std_msgs/msg/Int32MultiArray "data: [5, 0]"
ros2 topic pub /pedestrian_sign_present std_msgs/msg/Bool "data: false"
ros2 topic pub /pedestrian_motion geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0}}"
ros2 topic pub /pedestrian_ttc std_msgs/msg/Float64 "data: 1.8"  # CRITICAL!
```

**Expected Output:**
```bash
# data: "STOP_YIELD"
# Log: "TTC Critical override: 1.8s"
```

**Pass Criteria:**
- ✅ State: STOP_YIELD
- ✅ Log shows TTC critical override
- ✅ Overrides all other logic

---

### Test 2.3: STOP_YIELD (Regulatory Sign)
**Objective:** Verify sign detection response

**Input:**
```bash
# Pedestrian holding STOP sign
ros2 topic pub /fusion_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 45]"
ros2 topic pub /pedestrian_sign_present std_msgs/msg/Bool "data: true"  # SIGN!
ros2 topic pub /pedestrian_motion geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}}"
ros2 topic pub /pedestrian_ttc std_msgs/msg/Float64 "data: 20.0"
```

**Expected Output:**
```bash
# data: "STOP_YIELD"
# Log: "Pedestrian holding regulatory sign"
```

**Pass Criteria:**
- ✅ State: STOP_YIELD
- ✅ Log shows sign detection

---

### Test 2.4: SLOW_CAUTION (Standing Still)
**Objective:** Verify standing pedestrian response

**Input:**
```bash
# Pedestrian standing (speed < 0.1 m/s)
ros2 topic pub /fusion_pedestrian_position std_msgs/msg/Int32MultiArray "data: [14, 85]"
ros2 topic pub /pedestrian_sign_present std_msgs/msg/Bool "data: false"
ros2 topic pub /pedestrian_motion geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.03}}"  # 0.058 m/s
ros2 topic pub /pedestrian_ttc std_msgs/msg/Float64 "data: 15.0"
```

**Expected Output:**
```bash
# data: "SLOW_CAUTION"
# Log: "Pedestrian standing still (speed: 0.06 m/s)"
```

**Pass Criteria:**
- ✅ State: SLOW_CAUTION
- ✅ Calculated speed < 0.1 m/s
- ✅ Patience timer reset

---

### Test 2.5: STOP_WATCH → CREEP_PASS (Patience Logic)
**Objective:** Verify waiting timeout for moving pedestrian

**Input:**
```bash
# Moving pedestrian (speed >= 0.1 m/s)
ros2 topic pub /fusion_pedestrian_position std_msgs/msg/Int32MultiArray "data: [8, 90]"
ros2 topic pub /pedestrian_sign_present std_msgs/msg/Bool "data: false"
ros2 topic pub /pedestrian_motion geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.4}}"  # 0.5 m/s
ros2 topic pub /pedestrian_ttc std_msgs/msg/Float64 "data: 8.0"
```

**Expected Timeline:**
```bash
t=0.0s: data: "STOP_WATCH" (timer started)
t=1.0s: data: "STOP_WATCH" (waiting)
t=2.1s: data: "CREEP_PASS" (timeout exceeded)
```

**Pass Criteria:**
- ✅ Initial state: STOP_WATCH
- ✅ Timer starts on first detection
- ✅ After 2.0s: Changes to CREEP_PASS
- ✅ Log shows elapsed time

---

### Test 2.6: STOP_YIELD (Stale Data Fail-Safe)
**Objective:** Verify sensor failure detection

**Setup:**
```bash
# Start node, publish initial data, then STOP publishing
ros2 topic pub /fusion_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 0]" --once
ros2 topic pub /pedestrian_sign_present std_msgs/msg/Bool "data: false" --once
ros2 topic pub /pedestrian_motion geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}}" --once
ros2 topic pub /pedestrian_ttc std_msgs/msg/Float64 "data: 10.0" --once

# Wait > 0.5 seconds
```

**Expected Output:**
```bash
# After 0.5s: data: "STOP_YIELD"
# Log: "Stale data - executing fail-safe stop"
```

**Pass Criteria:**
- ✅ After 0.5s: Switches to STOP_YIELD
- ✅ Fail-safe triggered
- ✅ Warning logged

---

### Test 2.7: Priority Override Test
**Objective:** Verify TTC overrides sign detection

**Input:**
```bash
# Both sign AND critical TTC (TTC should win - Priority 2 > Priority 3)
ros2 topic pub /fusion_pedestrian_position std_msgs/msg/Int32MultiArray "data: [5, 0]"
ros2 topic pub /pedestrian_sign_present std_msgs/msg/Bool "data: true"  # Sign present
ros2 topic pub /pedestrian_motion geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0}}"
ros2 topic pub /pedestrian_ttc std_msgs/msg/Float64 "data: 1.5"  # Critical TTC
```

**Expected Output:**
```bash
# data: "STOP_YIELD"
# Log: "TTC Critical override: 1.5s" (NOT sign message)
```

**Pass Criteria:**
- ✅ State: STOP_YIELD
- ✅ Log shows TTC override (higher priority)

---

## 3. Controller Tests

### Test 3.1: Stanley Controller - Waypoint Tracking
**Objective:** Verify path following

**Setup:**
```bash
ros2 launch autoshield_full stanley_controller_launch.py vehicle_name:=e4 desired_speed:=2.0
```

**Requirements:**
- Joystick connected
- GPS receiving fix
- Waypoint file exists

**Test Procedure:**
1. Enable autonomous mode (LB + RB)
2. Drive around track
3. Monitor cross-track error

**Pass Criteria:**
- ✅ Vehicle follows waypoints
- ✅ Cross-track error < 1.0m
- ✅ Heading error < 10°
- ✅ Speed maintained at 2.0 m/s ±0.5

**Monitor:**
```bash
ros2 topic echo /pacmod/steering_cmd
ros2 topic echo /pacmod/accel_cmd
```

---

### Test 3.2: Safety Controller - CRUISE State
**Objective:** Verify normal driving (CRUISE mode)

**Setup:**
```bash
# Terminal 1: Start safety controller
ros2 launch autoshield_full straight_path_launch.py vehicle_name:=e4

# Terminal 2: Publish CRUISE state
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'CRUISE'"
```

**Test Procedure:**
1. Enable autonomous mode (LB + RB)
2. Publish CRUISE state
3. Monitor speed convergence to desired_speed (2.0 m/s)

**Pass Criteria:**
- ✅ Steering stays at 0° (±5°)
- ✅ Speed converges to 2.0 m/s within 10s
- ✅ Throttle active, brake at 0.0
- ✅ Log shows: "State: CRUISE | Target: 2.0 m/s"

**Monitor:**
```bash
ros2 topic echo /pacmod/vehicle_speed_rpt
ros2 topic echo /pacmod/accel_cmd
ros2 topic echo /pacmod/brake_cmd
```

---

### Test 3.3: Safety Controller - SLOW_CAUTION State
**Objective:** Verify slowing down for standing pedestrian

**Setup:**
```bash
ros2 launch autoshield_full straight_path_launch.py vehicle_name:=e4
```

**Input:**
```bash
# Vehicle at 3.0 m/s, switch to SLOW_CAUTION
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'SLOW_CAUTION'"
```

**Expected Behavior:**
- Vehicle slows from current speed to 2.5 m/s (slow_speed)
- PID-controlled deceleration
- Brake applied if needed

**Pass Criteria:**
- ✅ Speed converges to 2.5 m/s
- ✅ Smooth deceleration (no abrupt braking)
- ✅ Log shows: "State: SLOW_CAUTION | Target: 2.5 m/s"
- ✅ Brake engaged if speed > target

**Monitor:**
```bash
ros2 topic echo /pacmod/brake_cmd
```

---

### Test 3.4: Safety Controller - STOP_WATCH State
**Objective:** Verify stopping for moving pedestrian

**Setup:**
```bash
ros2 launch autoshield_full straight_path_launch.py vehicle_name:=e4
```

**Input:**
```bash
# Vehicle at speed, switch to STOP_WATCH
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'STOP_WATCH'"
```

**Expected Behavior:**
- Vehicle decelerates to 0.0 m/s
- PID-controlled braking
- Holding brake applied when speed < 0.1 m/s

**Pass Criteria:**
- ✅ Speed → 0.0 m/s
- ✅ Throttle: 0.0
- ✅ Brake increases as vehicle slows
- ✅ When stopped: brake = 0.3 (holding_effort)
- ✅ Log shows: "State: STOP_WATCH | Target: 0.0 m/s"

**Monitor:**
```bash
ros2 topic echo /pacmod/brake_cmd
# Should show: 0.3 when vehicle stopped
```

---

### Test 3.5: Safety Controller - CREEP_PASS State
**Objective:** Verify slow passing after patience timeout

**Setup:**
```bash
ros2 launch autoshield_full straight_path_launch.py vehicle_name:=e4
```

**Input:**
```bash
# From stopped, switch to CREEP_PASS
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'CREEP_PASS'"
```

**Expected Behavior:**
- Vehicle accelerates gently to 1.0 m/s (creep_speed)
- Smooth, cautious acceleration
- Maintains creep speed

**Pass Criteria:**
- ✅ Speed converges to 1.0 m/s
- ✅ Gentle acceleration (no jerking)
- ✅ Throttle limited by PID
- ✅ Log shows: "State: CREEP_PASS | Target: 1.0 m/s"

**Monitor:**
```bash
ros2 topic echo /pacmod/accel_cmd
```

---

### Test 3.6: Safety Controller - STOP_YIELD Emergency Stop
**Objective:** Verify emergency braking (CRITICAL TEST)

**Setup:**
```bash
ros2 launch autoshield_full straight_path_launch.py vehicle_name:=e4
```

**Input:**
```bash
# Vehicle at speed, trigger EMERGENCY STOP
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'STOP_YIELD'"
```

**Expected Behavior:**
- **IMMEDIATE** hard brake (bypasses PID!)
- Throttle = 0.0
- Brake = 0.6 (hard_brake_effort)
- No gradual deceleration

**Pass Criteria:**
- ✅ Throttle instantly → 0.0
- ✅ Brake instantly → 0.6
- ✅ No PID calculations (emergency mode)
- ✅ Log shows: "🚨 EMERGENCY STOP! State: STOP_YIELD | Applying hard brake: 0.60"
- ✅ Vehicle stops as quickly as possible

**Monitor:**
```bash
ros2 topic echo /pacmod/brake_cmd
# Should immediately show: 0.6
```

**⚠️ CRITICAL: This is a safety-critical test. Ensure:**
- Emergency stop happens within 1 control cycle (50ms @ 20Hz)
- No delay in brake application
- Vehicle stops in minimum distance

---

### Test 3.7: Safety Controller - State Transition Testing
**Objective:** Verify smooth state transitions

**Test Sequence:**
```bash
# Start at CRUISE
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'CRUISE'" --once
sleep 5

# Slow down
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'SLOW_CAUTION'" --once
sleep 5

# Stop and watch
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'STOP_WATCH'" --once
sleep 5

# Creep pass
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'CREEP_PASS'" --once
sleep 5

# Back to cruise
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'CRUISE'" --once
```

**Pass Criteria:**
- ✅ All transitions smooth (no jerking)
- ✅ PID doesn't wind up during stops
- ✅ Speed converges to each target
- ✅ Logs show each state change
- ✅ No oscillation between states

---

### Test 3.8: Safety Controller - Holding Brake Logic
**Objective:** Verify holding brake prevents roll

**Setup:**
```bash
ros2 launch autoshield_full straight_path_launch.py vehicle_name:=e4
```

**Test Procedure:**
1. Stop vehicle (STOP_WATCH state)
2. Wait until speed < 0.1 m/s
3. Monitor brake command

**Expected Behavior:**
```bash
# When speed > 0.1 m/s: PID-controlled brake
ros2 topic echo /pacmod/brake_cmd
# data: 0.45 (example - PID output)

# When speed < 0.1 m/s: Holding brake
ros2 topic echo /pacmod/brake_cmd
# data: 0.3 (holding_effort - overrides PID)
```

**Pass Criteria:**
- ✅ Brake transitions from PID to holding_effort
- ✅ Brake = 0.3 when stopped
- ✅ Vehicle doesn't roll
- ✅ Brake stays at 0.3 while stopped

---

### Test 3.9: Safety Controller - Brake vs Throttle Logic
**Objective:** Verify PID correctly uses brake OR throttle (never both)

**Test Cases:**

#### Case A: Need to Accelerate (pid_output > 0)
```bash
# Current: 1.0 m/s, Target: 2.0 m/s
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'CRUISE'"
```
**Expected:**
- throttle_cmd > 0.0 (e.g., 0.5)
- brake_cmd = 0.0

#### Case B: Need to Decelerate (pid_output < 0)
```bash
# Current: 3.0 m/s, Target: 1.0 m/s
ros2 topic pub /safety_decision std_msgs/msg/String "data: 'CREEP_PASS'"
```
**Expected:**
- throttle_cmd = 0.0
- brake_cmd > 0.0 (e.g., abs(pid_output))

#### Case C: At Target (pid_output ≈ 0)
```bash
# Current: 2.0 m/s, Target: 2.0 m/s (within dead zone)
```
**Expected:**
- throttle_cmd = 0.0
- brake_cmd = 0.0

**Pass Criteria:**
- ✅ NEVER both throttle AND brake active simultaneously
- ✅ Brake clamped to 0.0-1.0 range
- ✅ Throttle clamped to 0.0-max_accel
- ✅ Dead zone (±0.05 m/s) prevents oscillation

---

### Test 3.10: Safety Controller - Parameter Tuning Verification
**Objective:** Verify all parameters load correctly

**Setup:**
```bash
# Check parameters loaded
ros2 param get /autoshield_safety_controller desired_speed
ros2 param get /autoshield_safety_controller speeds.slow_speed
ros2 param get /autoshield_safety_controller speeds.creep_speed
ros2 param get /autoshield_safety_controller braking.hard_brake_effort
ros2 param get /autoshield_safety_controller braking.holding_effort
```

**Expected Output:**
```yaml
desired_speed: 2.0
speeds.slow_speed: 2.5
speeds.creep_speed: 1.0
braking.hard_brake_effort: 0.6
braking.holding_effort: 0.3
```

**Pass Criteria:**
- ✅ All parameters accessible
- ✅ Values match configuration file
- ✅ Can be changed at runtime
- ✅ Node responds to parameter updates

---

### Test 3.11: Emergency Stop (Joystick Disable)
**Objective:** Verify manual override

**Test Procedure:**
1. Enable autonomous mode
2. Vehicle driving
3. Press LB only (disable)

**Pass Criteria:**
- ✅ Vehicle immediately stops commanding
- ✅ PACMod disabled
- ✅ Log shows "Vehicle disabled"

---

## 4. Integration Tests

### Test 4.1: Full Perception Pipeline
**Objective:** Verify Lidar → Fusion → Decision → Control

**Setup:**
```bash
ros2 launch autoshield_full autoshield_full_launch.py \
  vehicle_name:=e4 \
  enable_lidar:=true \
  enable_fusion:=true \
  enable_high_level:=true \
  enable_straight_path:=true
```

**Simulated Scenario:**
```bash
# Person walking across path
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [8, 90]"
ros2 topic pub /rgbd_pedestrian_position std_msgs/msg/Int32MultiArray "data: [8, 92]"
ros2 topic pub /pedestrian_sign_present std_msgs/msg/Bool "data: false"
ros2 topic pub /pedestrian_motion geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0}}"
ros2 topic pub /pedestrian_ttc std_msgs/msg/Float64 "data: 5.0"
```

**Expected Flow:**
1. Lidar + Camera → Fused detection [8, 91]
2. Decision node → STOP_WATCH (moving person)
3. After 2s → CREEP_PASS
4. Controller → Slow speed command

**Pass Criteria:**
- ✅ Fusion produces combined detection
- ✅ Decision changes appropriately
- ✅ Controller responds to decision
- ✅ No crashes or errors

---

### Test 4.2: Multi-Pedestrian Scenario
**Objective:** Verify handling of multiple people

**Input:**
```bash
# 3 people: 2 matched + 1 lidar-only
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 45, 15, 90, 20, 180]"
ros2 topic pub /rgbd_pedestrian_position std_msgs/msg/Int32MultiArray "data: [11, 47, 16, 92]"

# Closest person moving fast
ros2 topic pub /pedestrian_motion geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.5}}"
ros2 topic pub /pedestrian_ttc std_msgs/msg/Float64 "data: 3.0"
```

**Expected:**
- Fusion: 3 detections output
- Decision: STOP_WATCH (closest person moving)

**Pass Criteria:**
- ✅ All detections processed
- ✅ Decision based on closest/most critical
- ✅ System remains stable

---

## 5. Safety-Critical Tests

### Test 5.1: Sudden Pedestrian Appearance
**Objective:** Test reaction time

**Procedure:**
1. System in CRUISE mode
2. Suddenly publish critical TTC

**Input:**
```bash
# t=0: No pedestrian
ros2 topic pub /fusion_pedestrian_position std_msgs/msg/Int32MultiArray "data: []" --once

# t=1: Pedestrian appears with TTC=1.0s!
ros2 topic pub /fusion_pedestrian_position std_msgs/msg/Int32MultiArray "data: [3, 0]"
ros2 topic pub /pedestrian_ttc std_msgs/msg/Float64 "data: 1.0"
```

**Pass Criteria:**
- ✅ State changes to STOP_YIELD within 1 cycle (50ms)
- ✅ Immediate brake command sent
- ✅ No hesitation

---

### Test 5.2: Sensor Dropout Recovery
**Objective:** Test fail-safe and recovery

**Procedure:**
1. Normal operation
2. Stop publishing sensor data
3. Resume data

**Timeline:**
```bash
t=0-2s: Normal data flow
t=2-3s: STOP publishing (trigger fail-safe)
t=3+:   Resume publishing
```

**Pass Criteria:**
- ✅ Fail-safe triggers at t=2.5s
- ✅ STOP_YIELD maintained during dropout
- ✅ System recovers when data resumes
- ✅ No lingering errors

---

### Test 5.3: Sign Detection Priority
**Objective:** Verify regulatory compliance

**Input:**
```bash
# Pedestrian standing with STOP sign
ros2 topic pub /fusion_pedestrian_position std_msgs/msg/Int32MultiArray "data: [12, 0]"
ros2 topic pub /pedestrian_sign_present std_msgs/msg/Bool "data: true"
ros2 topic pub /pedestrian_motion geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}}"
ros2 topic pub /pedestrian_ttc std_msgs/msg/Float64 "data: 50.0"
```

**Pass Criteria:**
- ✅ STOP_YIELD even though standing (would normally be SLOW_CAUTION)
- ✅ Sign overrides behavioral logic
- ✅ Vehicle remains stopped

---

## 6. Edge Cases & Failure Modes

### Test 6.1: Invalid Data Handling
**Objective:** Verify robustness to bad data

**Test Cases:**
```bash
# Negative distance
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [-5, 90]"

# Invalid angle (>360°)
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 400]"

# Odd-length array (missing angle)
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 90, 15]"
```

**Pass Criteria:**
- ✅ No crashes
- ✅ Invalid data filtered out
- ✅ Warnings logged
- ✅ System continues operating

---

### Test 6.2: GPS Loss During Waypoint Tracking
**Objective:** Test GPS failure handling

**Setup:**
```bash
# Start Stanley controller
# Unplug GPS during operation
```

**Pass Criteria:**
- ✅ Stale position data detected
- ✅ Vehicle stops safely
- ✅ Error logged

---

### Test 6.3: Rapid State Changes
**Objective:** Test state machine stability

**Input:**
```bash
# Rapidly toggle between states
# Person appears/disappears every 0.5s
```

**Pass Criteria:**
- ✅ No state oscillation
- ✅ Patience timer handles correctly
- ✅ No memory leaks
- ✅ System remains responsive

---

### Test 6.4: Maximum Detection Load
**Objective:** Test performance with many detections

**Input:**
```bash
# 20 pedestrians detected
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray \
  "data: [5,0, 6,10, 7,20, 8,30, 9,40, 10,50, 11,60, 12,70, 13,80, 14,90,
          15,100, 16,110, 17,120, 18,130, 19,140, 20,150, 21,160, 22,170, 23,180, 24,190]"
```

**Pass Criteria:**
- ✅ All detections processed
- ✅ Decision loop maintains 20 Hz
- ✅ No performance degradation
- ✅ Memory usage stable

---

### Test 6.5: Angle Wrapping Edge Cases
**Objective:** Test boundary conditions

**Test Cases:**
```bash
# Exactly 0°
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 0]"

# Exactly 180°
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 180]"

# Exactly 359°
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 359]"

# 0° + 180° average should be 90°, not 0°
ros2 topic pub /lidar_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 0]"
ros2 topic pub /rgbd_pedestrian_position std_msgs/msg/Int32MultiArray "data: [10, 180]"
```

**Pass Criteria:**
- ✅ Boundary angles handled correctly
- ✅ Averaging produces sensible results
- ✅ No angle discontinuities

---

## 📊 Testing Checklist

Use this checklist to track your testing progress:

### Sensor Fusion
- [ ] Test 1.1: Perfect Match
- [ ] Test 1.2: Lidar-Only
- [ ] Test 1.3: Camera-Only
- [ ] Test 1.4: Multiple Detections
- [ ] Test 1.5: Empty Arrays
- [ ] Test 1.6: Angle Wrapping
- [ ] Test 1.7: Matching Threshold

### High-Level Decision
- [ ] Test 2.1: CRUISE
- [ ] Test 2.2: TTC Critical
- [ ] Test 2.3: Sign Detection
- [ ] Test 2.4: Standing Still
- [ ] Test 2.5: Patience Logic
- [ ] Test 2.6: Stale Data
- [ ] Test 2.7: Priority Override

### Controllers
- [ ] Test 3.1: Stanley Tracking
- [ ] Test 3.2: Straight Path
- [ ] Test 3.3: Emergency Stop

### Integration
- [ ] Test 4.1: Full Pipeline
- [ ] Test 4.2: Multi-Pedestrian

### Safety-Critical
- [ ] Test 5.1: Sudden Appearance
- [ ] Test 5.2: Sensor Dropout
- [ ] Test 5.3: Sign Priority

### Edge Cases
- [ ] Test 6.1: Invalid Data
- [ ] Test 6.2: GPS Loss
- [ ] Test 6.3: Rapid Changes
- [ ] Test 6.4: Max Load
- [ ] Test 6.5: Angle Wrapping

---

## 🛠️ Testing Tools

### Monitor Multiple Topics
```bash
# Create a monitoring script
ros2 topic echo /fusion_pedestrian_position &
ros2 topic echo /safety_decision &
ros2 topic echo /pacmod/steering_cmd &
ros2 topic echo /pacmod/accel_cmd &
```

### Record Test Session
```bash
ros2 bag record -a -o test_session_$(date +%Y%m%d_%H%M%S)
```

### Replay Test Data
```bash
ros2 bag play test_session.bag
```

### Performance Monitoring
```bash
ros2 topic hz /fusion_pedestrian_position
ros2 topic bw /fusion_pedestrian_position
ros2 topic delay /fusion_pedestrian_position
```

---

## 📝 Test Report Template

```markdown
## Test: [Test ID and Name]
**Date:** YYYY-MM-DD
**Tester:** [Name]
**Vehicle:** [e2/e4]

### Setup
- ROS2 Version:
- Weather Conditions:
- Location:

### Results
- Pass/Fail:
- Observations:
- Metrics:

### Issues Found
1. [Description]
2. [Description]

### Notes
[Additional observations]
```

---

## 🚨 Critical Safety Checks (Must Pass Before Field Testing)

1. **Fail-Safe Verification**
   - Stale data triggers STOP_YIELD ✓
   - Sensor dropout triggers STOP_YIELD ✓
   - Manual override works instantly ✓

2. **TTC Critical Response**
   - TTC < 2.5s triggers immediate stop ✓
   - Overrides all other logic ✓
   - Response time < 100ms ✓

3. **Sign Detection**
   - Regulatory signs respected ✓
   - Overrides behavioral logic ✓

4. **Emergency Stop**
   - Joystick disable works ✓
   - Brake applied immediately ✓

---

**Good Luck Testing! 🎉**
