# Fleet Management – Group Work

VDA 5050-based fleet management for two mobile robots with vehicle-type-specific path constraints. Hello

> **Note:** The task structure in this README is a guide — a set of hints and recommendations
> on what to think about and in what order to approach the implementation.
> **The intermediate steps are not individually checked; the end result counts**.
> Teams are free to deviate from this structure as long as the final system works.
> Tasks may also be worked on in parallel by different team members.

---

## Goal

Develop a fleet manager as a team that coordinates **two real mobile robots** —
cat001 (Longitudinal Conveyor) and mouse001 (Lateral Conveyor) — in the IMRL test area.
Both robots must execute the given transportation tasks in the specified order in parallel,
**transporting packages between the transfer stations** using vehicle-type-specific paths and
docking orientations, station locking, and avoiding collisions and deadlocks.

---

## General Notes

**Implementation:**
- Build on the individual work codebase and expand it as a team.
- The fleet manager should be **modular and scalable** so that additional robots can be
  integrated with minimal code changes.
- Use the simulation to test implemented functionality before testing on real robots.
- **The `lif_file.json` is given and must not be changed.**
  Other input files (e.g. `agentsInitialization_file.json`, `transportationTasks_file.json`)
  may be modified for testing, but file names must remain unchanged.
- Transportation tasks must be assigned and executed **in the given order**.

**Real Robots:**
- To switch to real robots, set `"fleet_management_mode": "REAL_AGENTS"` and change
  `"mqtt_broker_ip"` from `"localhost"` to `"172.22.222.238"` (the IP address of the
  provided MQTT broker) in `data/input_files/config_file.json`.
- The velocity parameters `agentVelocity` and `agentRotationVelocity` as well as timing
  parameters such as `loading_unloading_time` and `init_fine_positioning_time` can be
  adjusted to match the observed real-robot behaviour. This enables more accurate planning.
- Real-world operation is inherently non-deterministic (e.g. a robot may be temporarily
  blocked). The fleet manager must handle such delays gracefully — they must not cause a
  crash, a collision, or a deadlock.

**Coordination:**
- Coordinate with the vehicle teams to ensure correct real-world integration.

**Final Event:**
- Two real mobile robots will be controlled on the test area.
- Transportation tasks for the final event will be provided on the day.
- **The robots may start at any position on the test area** — there are no fixed default
  start positions.

---

## Layout Overview

The layout is defined in `lif_file.json` and **cannot be changed**.
It contains **16 nodes**, **23 edges**, and **6 stations** on level `IMRL_TestArea`.

![Layout Graph](docs/visualizations/fig1_layout_graph.png)

LongC = `Longitudinal_Conveyor` (`cat001`),  LatC = `Lateral_Conveyor` (`mouse001`)

![Orientation Convention](docs/visualizations/fig2_orientations.png)

### Nodes

| Node | Role                                    | Position (x, y) | Notes                                            |
|------|-----------------------------------------|-----------------|--------------------------------------------------|
| N1A  | Station S1 (TRANSFER), interaction 1   | (5.61, 4.73)    | θ = 0.0 (LongC) / 1.57 (LatC)                   |
| N1B  | Station S1 (TRANSFER), interaction 2   | (6.97, 4.73)    | θ = 3.14 (LongC) / 1.57 (LatC)                  |
| N2   | Station S2 (TRANSFER)                  | (2.33, 1.42)    | θ = −1.57 (LongC) / 0.0 (LatC)                  |
| N3   | Station S3 (TRANSFER)                  | (5.82, 0.94)    | θ = −1.57 (LongC) / 0.0 (LatC)                  |
| N4   | Station S4 (PROCESS)                   | (0.75, 0.75)    | θ = None                                         |
| N5   | Station S5 (PROCESS)                   | (2.33, 4.71)    | θ = None                                         |
| N6   | Station S6 (PROCESS)                   | (7.90, 2.25)    | θ = None                                         |
| N7   | Dwelling C1 (CHARGING)                 | (1.00, 4.71)    | —                                                |
| N8   | Intermediate (fine-pos approach)       | (1.00, 3.45)    | LatC: `init_fine_positioning` to S2 (N2)         |
| N9   | Intermediate (fine-pos approach)       | (2.33, 3.45)    | LongC: `init_fine_positioning` to S2 (N2)        |
| N10  | Intermediate (fine-pos approach)       | (4.83, 3.60)    | LongC+LatC: `init_fine_positioning` to S1 (N1A)  |
| N11  | Intermediate (fine-pos approach)       | (4.50, 2.20)    | LatC: `init_fine_positioning` to S3 (N3)         |
| N12  | Dwelling C2 (CHARGING)                 | (7.50, 0.94)    | —                                                |
| N13  | Intermediate                           | (3.58, 3.45)    | —                                                |
| N14  | Intermediate (fine-pos approach)       | (5.82, 2.20)    | LongC: `init_fine_positioning` to S3 (N3)        |
| N15  | Intermediate (fine-pos approach)       | (7.75, 3.60)    | LongC+LatC: `init_fine_positioning` to S1 (N1B)  |

### Stations

| Station | Description                     | Interaction Node(s)  |
|---------|---------------------------------|----------------------|
| S1      | TRANSFER (Transfer_Station_1)   | **N1A, N1B**         |
| S2      | TRANSFER (Transfer_Station_2)   | N2                   |
| S3      | TRANSFER (Transfer_Station_3)   | N3                   |
| S4      | PROCESS (Process_Station_1)     | N4                   |
| S5      | PROCESS (Process_Station_2)     | N5                   |
| S6      | PROCESS (Process_Station_3)     | N6                   |
| C1      | CHARGING (dwelling)             | N7                   |
| C2      | CHARGING (dwelling)             | N12                  |

---

## Vehicle Types and Path Constraints

Two vehicle types operate in the layout:

| Agent      | Vehicle Type               | Abbreviation |
|------------|----------------------------|--------------|
| `cat001`   | `Longitudinal_Conveyor`    | LongC        |
| `mouse001` | `Lateral_Conveyor`         | LatC         |

### Vehicle-Type-Restricted Edges

Most edges are usable by both vehicle types.
The following edges are **restricted to one vehicle type only** (or define a required approach for both types)
and serve as the approach edges to TRANSFER stations.
All of them carry `"orientationType": "TANGENTIAL"` — see the Edge Orientations section below.

| Edge | From | To   | Allowed Type | Description                                    |
|------|------|------|--------------|------------------------------------------------|
| E1   | N9   | N2   | LongC only   | LongC approach to S2                           |
| E5   | N11  | N3   | LatC only    | LatC approach to S3                            |
| E8   | N8   | N2   | LatC only    | LatC approach to S2                            |
| E11  | N10  | N1A  | Both types   | Approach to S1 (interaction node N1A)          |
| E17  | N15  | N1B  | Both types   | Approach to S1 (interaction node N1B)          |
| E21  | N14  | N3   | LongC only   | LongC approach to S3                           |

> **Why these restrictions?**  Each vehicle type can only dock at a TRANSFER station
> from a specific direction determined by its conveyor orientation.
> The edge restrictions in the LIF enforce this: for each interaction node of each
> TRANSFER station there is exactly one valid approach edge per vehicle type.
> A* must filter edges by vehicle type to automatically find the correct approach route.

### Docking Orientations at Stations

The required docking orientation (theta) is vehicle-type-specific and is defined in
`lif_file.json` under `vehicleTypeNodeProperties`. It must be included in the order
message node for each station visit.

| Station | Node | θ for LongC  | θ for LatC   |
|---------|------|--------------|--------------|
| S1      | N1A  | 0.0 rad      | 1.57 rad     |
| S1      | N1B  | 3.14 rad     | 1.57 rad     |
| S2      | N2   | −1.57 rad    | 0.0 rad      |
| S3      | N3   | −1.57 rad    | 0.0 rad      |

At PROCESS stations S4 (N4), S5 (N5), and S6 (N6) theta is `None` — no specific docking
orientation is required. At intermediate and dwelling nodes theta is also `None`.

> **Note on `theta` as string `"None"`:** In `lif_file.json`, theta is stored as the
> string `"None"` when not specified. Always convert to Python `None` before using in
> order message construction.

### Vehicle-Specific Docking Positions and Fine Positioning

The physical docking position of a robot at a TRANSFER station deviates slightly from the node position defined in the LIF and varies by a few centimetres depending on the vehicle type (due to vehicle geometry).

#### `init_fine_positioning` — Defined on Approach Nodes in the LIF

The `init_fine_positioning` action is defined in the LIF under `vehicleTypeNodeProperties`
of the **approach node** (one node before the TRANSFER station). It is vehicle-type-specific:
a node may define the action for LongC only, LatC only, or both. When building the order
message, the fleet manager must check whether the current node carries this action for the
vehicle's type **and the next node in the path is the TRANSFER station interaction node**,
and if so, include it in the node's `actions` list.

The action carries the following `actionParameters` (as flat key-value pairs in the LIF):

| Parameter              | Meaning                                                              |
|------------------------|----------------------------------------------------------------------|
| `init_fine_pos_name`   | Name of the target station (e.g. `"station002_A"`)                  |
| `init_fine_pos_x`      | Absolute x-position of the target interaction node on the map        |
| `init_fine_pos_y`      | Absolute y-position of the target interaction node on the map        |
| `fine_pos_control_x`   | Control parameter x (see coordinate system below)                   |
| `fine_pos_control_y`   | Control parameter y                                                  |
| `fine_pos_control_theta` | Control parameter theta                                            |

#### Coordinate System for `fine_pos_control_*` Parameters

The coordinate system is defined at the interaction node (origin) using the **right-hand rule**:

- **+x** — from the interaction node toward the station center.
- **+y** — 90° CCW from +x (right-hand rule) — to the **left** when facing the station.
- **theta** — CCW rotation from frontal docking; theta = 0 means the robot's nose points toward +x (toward the station).

![Fine Pos Coordinate System](docs/visualizations/fig3_fine_pos_coordinate_system.png)

Docking values for this layout:

| Vehicle | `fine_pos_control_x` | `fine_pos_control_y` | `fine_pos_control_theta` |
|---------|----------------------|----------------------|--------------------------|
| LongC   | 0.02 m               | 0.00 m               | 0.0 (frontal)            |
| LatC    | 0.08 m               | 0.00 m               | +1.57 or −1.57 (sign depends on docking side) |

These parameters are primarily used by the **motion control team** for accurate final
docking. The fleet manager reads them from the LIF and forwards them **unchanged** in the
order message.

### Edge Orientations (TANGENTIAL)

Edges E1, E5, E8, E11, E17, and E21 carry `"orientationType": "TANGENTIAL"` with
`"vehicleOrientation": 0.0` in `vehicleTypeEdgeProperties`. This means:

- The vehicle aligns its heading **tangentially to the edge** as it travels along it.
- The `vehicleOrientation` value (`0.0` in all cases in this layout) is a **constant
  offset** added to the tangential heading. It is always specified relative to the
  **LIF edge direction** (from `startNodeId` to `endNodeId`), **independent of the
  actual travel direction**.
- As a result, the **vehicle is always oriented toward the `endNodeId` of the LIF
  edge**, regardless of travel direction. Moving from `startNodeId` to `endNodeId`,
  the heading is directed toward `endNodeId` (forward movement). Moving in the reverse direction (from
  `endNodeId` to `startNodeId`), the vehicle still points toward `endNodeId` (backward movement).

The fleet manager sends orientation information in the order message;
trajectory planning is the responsibility of the motion control. The fleet
manager's responsibility is to ensure the robot reaches TRANSFER stations **via the
type-correct edge**, so that the vehicle's final heading matches the required docking
theta.

Since the approach edges are TANGENTIAL and the interaction node orientation is aligned
with the arriving edge direction, **robots are not permitted to rotate at interaction
nodes of TRANSFER stations** — they arrive already facing the correct docking direction and also have to leave the node in that orientation.

---

## Station Locking

Only **one agent** may interact with a station at any point in time.
When an agent occupies (or has reserved) any interaction node of a station, **all**
interaction nodes of that station are blocked for all other agents.

> **Example:** Station S1 has two interaction nodes: N1A and N1B. If one robot is
> assigned to N1A, N1B is also reserved and unavailable to the other robot.
> Both nodes are released together when the robot completes its task at S1.

This must be enforced by a shared `ResourceManager` (or equivalent) that the fleet
manager checks before releasing any order leading to an interaction node.
Actions at a station (`pick`, `drop`, `process`) are fully executed by the motion
control. The fleet manager must wait for successful task completion.

---

## Initialization

At startup, the robots do not know the graph — their state message contains an
**empty `lastNodeId`** and only reports the robot's physical position via the
`agvPosition` field. The fleet manager must safely initialize all robots before
beginning task assignment:

1. **Wait** for the first VDA 5050 state message from each robot.
   Read the robot's position from the `agvPosition` field.
2. **Find** the nearest graph node to the reported position.
   Use this as the robot's effective current node.
3. **Check** whether that node is a dwelling (CHARGING) node.
4. If it is **not** a dwelling node, plan a path to the nearest dwelling node and send
   an order to move there. Ensure no collision occurs during this relocation. 
   Wait for each robot to arrive before continuing.
5. Once all robots are on a dwelling node, begin regular task assignment.

---

## Transportation Tasks

Tasks are defined in `data/input_files/transportationTasks_file.json`.
The movement structure for a task is:

```
current position → pick station → process station(s) [0–3] → drop station
```

**Tasks must be executed in the given order.**  The task with the lowest index that
has not yet been assigned is always the next task to assign.
The task assignment criterion (which idle robot receives the next task) is defined
by the group — a reasonable default is to choose the idle robot whose current
position is geographically closest to the task's first station.

After completing a task the robot does not need to return to a dwelling node.
Whether the robot waits at the drop station or is sent to a dwelling node between
tasks depends on the implementation.

---

## Tasks

### Task 1: Complete the Individual Work

Ensure that the individual work (Tasks 1–10) is fully implemented and working.
Discuss your solutions in the group and merge them into a single shared codebase
that serves as the starting point for all group work tasks.

**Goal:** One simulated mobile robot executes all transportation tasks in the given
layout, fully automatically, using A*, VDA 5050 order messages, and state callbacks.

---

### Task 2: Extend the Code for a Second Mobile Robot

Extend the codebase to support `mouse001` (LatC) alongside `cat001` (LongC).
The key additions are: second agent initialization, vehicle-type-aware graph
representation, vehicle-type-aware A* pathfinding, and correct theta selection per
vehicle type in order messages.

> **Note:** Collisions between the two robots can be neglected in this task.
> Full collision and deadlock prevention is addressed in later tasks.

#### 2a — Add `cat001` to the Initialization File

**File:** `data/input_files/agentsInitialization_file.json`

Add a second agent entry for `cat001`. Place it anywhere on the map:

```json
{
    "agentId": "cat001",
    "vehicleTypeId": "Longitudinal_Conveyor",
    "stateTopic": "KIT/IMRL/cat001/state",
    "orderTopic": "KIT/IMRL/cat001/order",
    "visualizationTopic": "KIT/IMRL/cat001/visualization",
    "agentPosition": {"x": 7.50, "y": 0.94, "theta": 0.0},
    "agentVelocity": 1.5,
    "agentRotationVelocity": 3,
    "agentPositiveDirectionPreffered": true
}
```

#### 2b — Extend the `Agent` Class

**File:** `src/fleet_management/agents.py`

Add the `vehicle_type_id` attribute to `Agent`, read from `agentsInitialization_file.json`:

```python
self.vehicle_type_id = entry['vehicleTypeId']
# e.g. 'Lateral_Conveyor' for mouse001, 'Longitudinal_Conveyor' for cat001
```

In `get_agents()`, make sure the required entries from `agents_initialization_data['agents']`
are used.

#### 2c — Extend the `Graph` Class

**File:** `src/fleet_management/graph.py`

| Method | Change required |
|--------|----------------|
| `get_edges()` | Add `allowedVehicleTypes` (list of type ID strings from `vehicleTypeEdgeProperties`) to each edge dict |
| `get_connected_nodes(node_id)` | Add optional `vehicle_type_id` parameter; when provided, only return neighbours reachable via a compatible edge |
| `get_connected_edge(a, b)` | Add optional `vehicle_type_id` parameter; return `None` if no edge between `a` and `b` is compatible with that type |

#### 2d — Extend `PathPlanning.astar_search()`

**File:** `src/fleet_management/fleet_management.py`

Add a `vehicle_type_id` parameter so the search only expands edges the vehicle is
allowed to use:

```python
def astar_search(self, start_node: str, goal_node: str, vehicle_type_id: str) -> tuple:
    # use self.graph.get_connected_nodes(node_id, vehicle_type_id)
    # use self.graph.get_connected_edge(a, b, vehicle_type_id)
```

#### 2e — Update Order Message Construction

**File:** `src/fleet_management/fleet_management.py`

Update the three pipeline methods to accept and use `vehicle_type_id`:

| Method | Change |
|--------|--------|
| `build_path_for_task(task, start_node, vehicle_type_id)` | Pass `vehicle_type_id` to every `astar_search()` call |
| `build_order_nodes(path_nodes, task, vehicle_type_id)` | Use `self.graph.get_node_theta(node_id, vehicle_type_id)` instead of the hardcoded `Longitudinal_Conveyor` lookup |
| `build_order_edges(path_nodes, path_edges)` | No structural change needed; the correct edge IDs are already selected by the vehicle-type-aware A* |

In `fleet_manager()`, pass `agent.vehicle_type_id` through the pipeline for each agent.

**Goal:** Both simulated robots can plan and execute tasks. Each robot uses only
edges compatible with its vehicle type. Docking orientations at TRANSFER stations
are correct per vehicle type. Collisions can be neglected for now.

---

### Task 3: Develop a Simple Fleet Manager

Using the vehicle-type-aware infrastructure from Task 2, implement a simple fleet
manager that coordinates both robots **sequentially** (only one moves at a time).

**Program flow:**

1. **Initialization**
   On startup, if agents are not on a dwelling node, plan a path to the
   nearest dwelling node and send the agents there first. Make sure no collisions occure on the way
   and move the agents sequentially. Wait for confirmation that both agents have reached a dwelling node 
   before starting task assignment.

2. **Task assignment**
   Assign tasks strictly in order (T1 before T2, etc.).
   Choose the idle agent according to a defined criterion (e.g. closest to the
   task's first station). Only one agent is dispatched at a time; the other
   waits on its dwelling node.

3. **Path planning and order dispatch**
   Using the vehicle-type-aware A* from Task 2, plan a path for the assigned agent
   from its current position through the task stations and build the VDA 5050 order
   message.

   > **Design choice:** How you plan and dispatch the path is a key decision that
   > directly affects how collisions and deadlocks are prevented. Common approaches
   > include planning and sending the full route as a single order, or dispatching
   > it segment by segment (e.g. current node → pick station first, wait for arrival,
   > then plan the next leg). Each approach has different trade-offs for reactivity
   > and safety — the choice is up to your team.

4. **Monitoring and completion**
   Monitor agent state via `state_callback`. When the task is complete
   (`task_completed == True`, `agent_state == 'IDLE'`), assign the next task.

5. **Termination**
   Stop when all tasks have been completed.

**Goal:** Two simulated robots execute all transportation tasks in the given
order. Only one robot moves at a time. The initialization phase correctly handles
arbitrary starting positions. Neither collisions nor deadlocks occur.

---

### Task 4: Develop a Concept for a More Efficient Fleet Manager

Develop a **written concept** for a fleet manager where **both robots operate in
parallel**. The following questions are meant as guidance — you do not need to answer
all of them, but they cover the key design areas you should think through.

**Task Assignment:**
- Given the fixed task order constraint, how can both robots be kept busy in parallel?
- Which assignment criterion minimizes total task completion time?

**Collision and Deadlock Prevention:**
- How do you ensure that two robots never occupy the same node or edge simultaneously?
- What reservation or locking model do you use, and when are resources reserved and
  released?
- How do you detect and resolve deadlocks (e.g. two robots mutually waiting for each
  other's position)?
- How does your approach handle real-world delays (a robot moving slower than expected)?

**Path Planning Strategy:**
- When and how are paths planned — fully in advance, or incrementally as the robot moves?
- How does the planning strategy interact with collision prevention?
  (e.g. can a path only be planned if all required nodes/edges are free?)

**Station Locking:**
- With both robots working in parallel, how is station locking managed?
- Remember: all interaction nodes of a station are blocked simultaneously (see
  *Station Locking* section).

**Order Dispatch and Monitoring:**
- When are new order messages sent to the robots?
- How is the completion of individual actions (`pick`, `drop`, `process`) detected?
- How does the fleet manager react if a robot reports an error or unexpected state?

Document your concept clearly. Present it in Task 5.

---

### Task 5: Presentation of the Concept and the Collaboration (Milestone 2)

**Format:** Team presentation (≈ 10 minutes) with slides. All team members participate.

**Required content:**
- **Team & Roles:** Introduce team members and their responsibilities.
- **Collaboration:** Describe your workflow, tools, and coordination methods.
- **Progress:** Show what has been implemented so far (code, simulation results).
- **Concept:** Present the fleet management concept from Task 5 and justify key design choices.
- **Goals:** Define what the team aims to achieve by the final event.

> **Note:** You may not have completed all previous tasks by the milestone date.
> A well-developed concept for Task 4 is the most important deliverable here.

---

### Task 6: Collaborate with the Vehicle Teams

Working with the vehicle teams, determine which VDA 5050 parameters need to be
exchanged during operation:

- **Order message:** Which fields are relevant for each team?
  Pay particular attention to actions (`init_fine_positioning`, `pick`, `drop`, `process`)
  and to theta at TRANSFER station nodes.
- **State message:** Which fields does the fleet manager rely on?
  (`lastNodeId`, `nodeStates`, `edgeStates`, `actionStates`, `agvPosition`)

Define and test the **initialization phase** with the vehicle teams:
- How does the fleet manager determine a robot's starting node when the system launches?
- What should happen if a robot's starting position is not a dwelling node?

**Test with one real mobile robot first.**

**Goal:** One real mobile robot executes transportation tasks in the test area,
including correct `init_fine_positioning` + `pick`/`drop` sequences at TRANSFER stations
and correct `process` at PROCESS stations, with proper state monitoring.

---

### Task 7: Implement the Fleet Manager Concept

Implement the fleet manager concept from Task 5 as a team.
Divide the work into logical sub-areas and implement incrementally.

**Implementation checklist (no claim to completeness):**
- [ ] Resource manager (node/edge reservation, station locking) — consider a new
  file `src/fleet_management/resource_manager.py`
- [ ] Parallel task execution with thread-safe state sharing
- [ ] Deadlock detection and resolution strategy
- [ ] Delay handling (e.g. fixed reservation order or timeout-based replanning)

**Verify the following:**
- **Efficiency:** How much faster does parallel execution complete all tasks
  compared to the sequential approach from Task 3?
- **Collision-free operation:** Do collisions or deadlocks occur during simulation?

Document the implementation and test results.

**Goal:** Two simulated robots collaborate in parallel to execute all transportation
tasks efficiently. Vehicle-type constraints, station locking, and task ordering are
enforced. Neither collisions nor deadlocks occur.

---

### Task 8: Test the Fleet Manager in the Real-World

Test the fleet manager with the real mobile robots in the test area.

- Adjust `agentVelocity` and `agentRotationVelocity` in `config_file.json` to match
  the observed behaviour of the real robots.
- Start with one mobile robot; add the second only when single-robot execution works.
- Verify that the initialization phase correctly works for arbitrary start positions of the robots.
- Test that `init_fine_positioning` → `pick`/`drop` sequences work correctly at
  TRANSFER stations for both vehicle types.
- Test that station locking prevents both robots from docking at the same station simultaneously.

**Goal:** Two real robots collaborate to execute all transportation tasks in the
designed layout. Vehicle-type constraints, correct docking orientations, and station
locking are enforced in real-world conditions. Neither collisions nor deadlocks occur.

---

### Task 9: Showcase (Final Event)

**Purpose:** Demonstrate a complete, working fleet management system that enhances
manufacturing efficiency in the IMRL test area.

**Poster presentation:** Present the fleet management concept, design decisions,
and test results.

**Live demonstration:** The fleet manager coordinates two real mobile robots
executing transportation tasks provided on the day of the event.

**System requirements for the final event:**
- Two real robots from different vendors (two vehicle teams) are integrated via VDA 5050.
- The fleet manager handles arbitrary starting positions.
- Tasks are executed in the specified order.
- Correct docking orientations are used per vehicle type.
- Station locking prevents concurrent station conflicts.
- The system operates collision-free and deadlock-free throughout the demonstration.

> **Note:** During the final demonstration, temporal obstacles may appear in the test
> area that block robot paths and cause delays. The fleet manager must handle such
> situations gracefully — they must not cause a crash, a deadlock, or a collision.

---

## Important Notes

### VDA 5050 Protocol

- **Order message:** Sent by the fleet manager to command a robot.
  Nodes have **even** sequenceIds (0, 2, 4, …), edges have **odd** sequenceIds (1, 3, 5, …).
- **State message:** Sent by the robot to report its current status.
  `lastNodeId`, `nodeStates`, `edgeStates`, and `actionStates` are used by the fleet manager.
- `orderId` must be unique per order (use the `order_header_id` counter).
- `released: true` marks all nodes/edges that the robot is allowed to traverse.

### Actions

Actions are triggered by the fleet manager in the order message and executed
by the motion control on the robot. The fleet manager must wait for the action
to complete (all `actionStates` report `"FINISHED"`) before the robot proceeds.

| Action                | Where in path            | Who executes it              |
|-----------------------|--------------------------|------------------------------|
| `init_fine_positioning` | Approach node before TRANSFER station (defined in LIF per vehicle type) | Motion control (fine docking alignment) |
| `pick`                | TRANSFER station node    | Motion control (conveyor belt) |
| `drop`                | TRANSFER station node    | Motion control (conveyor belt) |
| `process`             | PROCESS station node     | Motion control (wait for `processingTime` s) |

### MQTT Topics

| Topic                          | Direction      | Content                        |
|-------------------------------|----------------|--------------------------------|
| `KIT/IMRL/mouse001/order`     | Fleet → Robot  | VDA 5050 order message         |
| `KIT/IMRL/mouse001/state`     | Robot → Fleet  | VDA 5050 state message         |
| `KIT/IMRL/cat001/order`       | Fleet → Robot  | VDA 5050 order message         |
| `KIT/IMRL/cat001/state`       | Robot → Fleet  | VDA 5050 state message         |
| `KIT/IMRL/tasks`              | Fleet → Viz    | Task list (for visualizer)     |

### Threading

Use Python's `threading.Thread` with `daemon=True` for background loops.
Protect shared data structures (reservation maps, agent states) with
`threading.Lock()` to avoid race conditions when two agents update state concurrently.
