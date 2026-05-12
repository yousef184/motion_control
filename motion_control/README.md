# Motion Control Simulation

Your task is to design and implement algorithms for planning and controlling the motion of a mobile robot and conveyors to fulfill orders from a management system, while considering realistic conditions such as noise and dynamic constraints.

In the simulation, you will develop a controller to realize the most fundamental function — moving to a desired goal. All other functions, including handling safety states, docking, and controlling conveyor belts for cargo transportation, will be practiced directly on our real mobile robots.

---

## How to Run

1. Open `run_motion_control_simulation.py` in the `imrl_workspace/motion_control/` directory.
2. Set the desired task number in that file (default: `1`, available: `1`–`3`):
   ```python
   args = ["--task", "1", ...]
   ```
3. Run it via the VS Code run button or in a terminal:
   ```bash
   python run_motion_control_simulation.py
   ```
4. The simulation window opens and an order is published. The released part of the route is visualized as a green path; unreleased nodes/edges appear in dark gray.

> **Tip:** Only one script can be executed with the VS Code run button. To run a second script simultaneously (e.g., your control script), use VS Code's "Run and Debug" feature or a separate terminal:
> ```bash
> python src/motion_control/task_1.py
> ```

---

## Project Structure

```
motion_control/
├── run_motion_control_simulation.py   Main entry point — starts the simulation
├── data/
│   ├── interface/
│   │   └── order.schema               VDA 5050 order message JSON schema
│   │   ├── state.schema               VDA 5050 state message JSON schema
│   ├── map1/                          Map used for Tasks 1–4
│   │   ├── LIF.json                   Layout Interchange Format — graph definition
│   │   ├── order_msg.json             Initial order message
│   │   ├── updated_order_msg.json     Second (unreleased) part of the order
│   │   └── static_map.json            Static obstacle map
│   ├── map5/                          (currently not necessary)
│   ├── map6/                          (currently not necessary)
│   ├── map7/                          (currently not necessary)
│   └── map_ifl/                       (currently not necessary)
└── src/
    └── motion_control/
        ├── task_1.py                  Task 1 — parse the order (starter code)
        ├── task_2_3.py                Tasks 2 & 3 — execute the order / handle noise (starter code)
        └── keyboard_control_commented.py (currently not necessary) 
```

---

## MQTT Topics

| Topic | Direction | Content |
|-------|-----------|---------|
| `KIT/IMRL/mouse001/order` | Simulation → Your Code | VDA 5050 order message |
| `KIT/IMRL/mouse001/pose`  | Simulation → Your Code | Current robot pose |
| `KIT/IMRL/mouse001/cmd`   | Your Code → Simulation | Velocity commands |
| `KIT/IMRL/mouse001/state` | Your Code → Simulation | VDA 5050 state message |

Example messages for all topics can be found in `imrl_workspace/json_examples/`.

---

## Version Control — One Branch Per Task

Each task builds on the previous one. To always have a working version to fall back to, **create one Git branch per task**:

```bash
# Start Task 1 from main
git checkout main
git checkout -b task/task-1

# ... implement Task 1 ...
git add src/motion_control/task_1.py
git commit -m "Task 1: order parsing implemented"

# Start Task 2 from Task 1
git checkout task/task-1
git checkout -b task/task-2
# ... continue for Tasks 2, 3
```

**Tip:** Commit at the end of every implemented feature, not only when everything works.

---

## Tasks

> **Minimum requirement for the individual work:** Complete **Tasks 0 – 3** (Milestone 1).

### Task 0 — Test the Simulation Environment (no implementation)

Before starting the individual tasks, verify that the simulation works.

1. Follow the "How to Run" section above to start the simulation.
2. Use the **arrow keys** to drive the robot. Make sure the simulation window is activated — otherwise key presses are not captured.
4. The robot's trajectory is shown in red. If the robot moves, your environment is ready.

---

### Task 1 — Parse the Order

**File:** `src/motion_control/task_1.py`

Parse the necessary information from the VDA 5050 order message and convert it into a **route** (an ordered list of waypoints).

- Orders are published on `KIT/IMRL/mouse001/order`.
- Only **released** nodes and edges (`"released": true`) should be included in the route.
- Released nodes and edges are visualized in green; unreleased ones in dark gray.

**Steps to implement in `Robot.receive_order()`:**
1. Decode the JSON payload.
2. Extract `nodes` and `edges` from the message.
3. Filter to only the released entries.
4. Store the waypoint positions for use in Task 2.

**Goal:** Print or store the sequence of released node positions `(x, y)` that form the route.

---

### Task 2 — Execute the Order
**remember to set the correct task in run_motion_control_simulation.py**
**File:** `src/motion_control/task_2_3.py`

Control the robot to drive along the extracted route, waypoint by waypoint.

- The robot's pose (without noise) is published on `KIT/IMRL/mouse001/pose`.
- Velocity commands must be published to `KIT/IMRL/mouse001/cmd`.
- Make sure the robot will drive through all released nodes.
- Only the released route may be executed.

**State reporting (required to unlock the second part of the order):**
- Build and publish a VDA 5050-compliant state message to `KIT/IMRL/mouse001/state`.
- The simulation releases the second part of the order only after receiving a correct state message.
- An example state message is in `imrl_workspace/json_examples/stateMessage_Example.json`.

**Steps to implement:**
1. `Robot.update_pose()` — parse the pose message and update `self.x`, `self.y`, `self.theta`.
2. `Robot.receive_order()` — build `self.trajectory` from released node positions.
3. `Robot.build_status_message()` — construct a VDA 5050 state dict.
4. `follow_trajectory()` — implement a controller (e.g., proportional) that computes `linear_vel` and `angular_vel`, advances `current_index` when close enough to the target, and sets `status_update_needed = True` on waypoint arrival.

**Evaluation:** The deviation from the assigned route is shown in the upper right corner of the simulation window. Minimize it.

---

### Task 3 — Handle Noise and Dynamic Constraints

**File:** `src/motion_control/task_2_3.py`

Adapt the controller from Task 2 to handle realistic conditions:

- **Execution noise and delay:** Gaussian noise is introduced to simulate imperfect robot execution, meaning that the robot may not follow commands exactly and command execution may be delayed.
- **Dynamic constraints:** The robot's acceleration, deceleration, and maximum velocity are limited — abrupt velocity changes are not possible.

**Goal:** Minimize the deviation from the assigned route under these conditions.

**Suggested adaptations:**
- Apply a low-pass filter or moving-average filter to smooth noisy pose readings.
- Implement velocity ramp-up/ramp-down to respect acceleration limits (e.g., gradually increase/decrease speed rather than jumping to maximum velocity immediately).
- Tune controller gains to avoid oscillation caused by noisy feedback.

---

## Troubleshooting

- **Robot does not move:** Verify the MQTT broker (Mosquitto) is running (`mosquitto -v`) and that your control script is connected to `localhost:1883`.
- **Keys not captured in keyboard control:** Click on the simulation window to give it focus before pressing arrow keys.
- **Simulation does not open:** Check that `run_simulation` executable has execute permission:
  ```bash
  chmod +x src/mobile_robot_simulation/dist/run_simulation
  ```
- **Second part of order never released:** Ensure your state message includes all required VDA 5050 fields. Compare with `json_examples/stateMessage_Example.json`.
