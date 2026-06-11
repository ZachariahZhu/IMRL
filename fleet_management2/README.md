# Fleet Management Simulation

A VDA 5050-based fleet management for a mobile robotics system.

---

## How to Run

1. Open a terminal in the **workspace root** (`imrl_workspace/`) and activate the conda environment:
```bash
conda activate imrl_env
```

2. Run the simulation:
```bash
python fleet_management/run_fleet_management_simulation.py
```

Alternatively, open `run_fleet_management_simulation.py` in VS Code and press the **Run** button (▶) — VS Code will use the configured Python interpreter automatically.

The script automatically starts the `agent_simulation`, waits for it to
initialize (default: 6 seconds), then starts the fleet management logic.

If the robot does not move after the simulation opens, try increasing the sleep
time in `run_fleet_management_simulation.py` (the `time.sleep(6)` after
`subprocess.Popen`).

---

## Project Structure

```
fleet_management/
├── data/
│   ├── input_files/
│   │   ├── lif_file.json                   Layout Interchange Format — graph definition
│   │   ├── config_file.json                MQTT broker address, simulation run time, etc.
│   │   ├── agentsInitialization_file.json  Initial robot position, velocity, topics
│   │   ├── transportationTasks_file.json   List of transportation tasks to execute
│   │   ├── orderMessage_Example.json       Example VDA 5050 order message
│   │   └── stateMessage_Example.json       Example VDA 5050 state message
│   └── output_files/
│       └── logging_file.log
├── src/
│   ├── run_fleet_management_simulation.py  Main entry point
│   ├── fleet_management/
│   │   ├── agents.py                       Agent digital twin (Tasks 5 & 8)
│   │   ├── fleet_management.py             Path planning & order dispatch (Tasks 6 & 7)
│   │   ├── graph.py                        Graph model from LIF (Task 4)
│   │   ├── task_assignment.py              Task-to-agent assignment (Task 9)
│   │   └── task_management.py              Loads transportation tasks from file
│   ├── vda5050_interface/
│   │   ├── interfaces/
│   │   │   └── order_interface.py          VDA 5050 order message builder (Task 3)
│   │   ├── json_schemas/
│   │   │   ├── order.schema
│   │   │   ├── state.schema
│   │   │   └── visualization.schema
│   │   └── mqtt_clients/
│   │       ├── mqtt_publisher.py
│   │       └── mqtt_subscriber.py
│   └── mobile_robot_simulation/
│       └── dist/
│           └── agent_simulation            Simulation executable
└── tests/
    ├── test_task3_order_message.py
    ├── test_task4_graph.py
    ├── test_task5_agents.py
    ├── test_task6_astar.py
    ├── test_task7_fleet_integration.py
    ├── test_task8_state_callback.py
    ├── test_task9_task_assignment.py
    └── test_task10_fleet_manager.py
```

---

## Version Control — One Branch Per Task

Each task builds on the previous one. To always have a working version to fall back to, **create one Git branch per task**.
All branches should be under your u-account namespace:

```bash
# Start Fleet Management Task 3 from main
git checkout main
git checkout -b u-account/fm-task-3

# ... implement Task 3 in src/fleet_management/fleet_management.py ...
git add src/fleet_management/fleet_management.py \
        src/vda5050_interface/interfaces/order_interface.py
git commit -m "Task 3: generate_order_message() implemented, path filled manually"

# Start Task 4 from Task 3
git checkout u-account/fm-task-3
git checkout -b u-account/fm-task-4
# ... continue for Tasks 5–10
```

**Tip:** Commit at the end of every implemented feature, not only when everything works.

---

## Layout Overview

The given layout (`lif_file.json`) contains **14 nodes** and **21 bidirectional edges**.

| Node | Role              | Position (x, y) |
|------|-------------------|-----------------|
| N1   | Station S1 (PROCESS)  | (2.46, 0.79) |
| N2   | Station S2 (TRANSFER) | (2.46, 3.70) |
| N3   | Station S3 (PROCESS)  | (5.45, 2.25) |
| N4   | Station S4 (TRANSFER) | (5.55, 4.75) |
| N5   | Dwelling (start)  | (1.05, 0.90) |
| N9   | Dwelling          | (4.70, 0.80) |
| N10  | Dwelling          | (1.80, 3.50) |
| N14  | Dwelling          | (6.80, 3.50) |
| N6–N8, N11–N13 | Intermediate nodes | — |

**Station types:**
- **TRANSFER** (S2/N2, S4/N4): pick and drop actions. The robot needs `init_fine_positioning`
  on the node immediately before a TRANSFER station, then `pick` or `drop` on the station node. 
  A transportation task always starts with a `pick` and ends with a `drop`.
- **PROCESS** (S1/N1, S3/N3): processing actions. The robot stops and waits for the specified
  `processingTime` (in seconds). There can be between 0 and 3 process steps between `pick` and `drop`.

The robot starts at the dwelling node **N5**.

---

## Transportation Tasks

Transportation tasks are defined in `data/input_files/transportationTasks_file.json`.
Each task contains an ordered list of **stations** the robot must visit.

**Required movement structure (must be implemented for every task):**

```
dwelling (start) → pick station → process station(s) [0–3] → drop station → dwelling (return)
```

- Every task starts with exactly one `pick` action at a **TRANSFER** station.
- Every task ends with exactly one `drop` action at a **TRANSFER** station.
- Between pick and drop there are 0 to 3 `process` stops at **PROCESS** stations.
- After the drop, the robot returns to the nearest dwelling node.
- The robot's starting position for the first task is dwelling node **N5**.
  For subsequent tasks it starts from the dwelling node it returned to after the previous task.

**Task file format:**

```json
{
  "transportationTaskId": "T1",
  "stations": [
    {"nodeId": "N4", "actionType": "pick"},
    {"nodeId": "N3", "actionType": "process", "processingTime": 1.0},
    {"nodeId": "N2", "actionType": "drop"}
  ]
}
```

The `stations` list covers only the waypoints, not the dwelling legs.
The fleet management code is responsible for adding the travel to/from the dwelling.

---

## Tasks

In the individual work, students implement an A* pathfinding algorithm and a VDA 5050 communication interface
to execute multi-stop transportation tasks automatically with one mobile robot.

> **Minimum requirement for the individual work: Complete Tasks 1 – 6**.

### Task 1 — Get an Overview of the Codebase. (no implementation)

Study the `fleet_management` project.
- Get an overview of the codebase and familiarize yourself with the structure.
- Understand how the input files are read into the `run_fleet_management_simulation.py` file.
- Make changes to the input files and test how the fleet management simulation works.

**Goal:** You are able to explain the `fleet_management` codebase with all it's components.

---

### Task 2 — Run the Simulation (no implementation)

Run `src/run_fleet_management_simulation.py` and observe the robot executing the
hardcoded example order from `data/input_files/orderMessage_Example.json`.

- Study `orderMessage_Example.json` carefully: note the sequenceId pattern, actions,
  and node positions.
- Study `data/input_files/stateMessage_Example.json` to understand the VDA 5050 state
  message format.

**Goal:** Understand the VDA 5050 order and state message formats before implementing
them yourself.

**VDA 5050 sequenceId rule (critical for Task 3):**
- Nodes receive **even** sequenceIds starting at 0: 0, 2, 4, 6, …
- Edges receive **odd** sequenceIds starting at 1: 1, 3, 5, 7, …
- They interleave perfectly: node(0) → edge(1) → node(2) → edge(3) → …

---

### Task 3 — Implement `generate_order_message()`

**Files:** `src/fleet_management/fleet_management.py` and
`src/vda5050_interface/interfaces/order_interface.py`

This task has two parts that together replace the hardcoded example from Task 2:

**Part A — Manually fill the path in `fleet_manager()`**
(`src/fleet_management/fleet_management.py`)

In the `fleet_manager()` method, manually fill the `nodes` and `edges` lists
with the route for the first transportation task (T1).
The journey must follow the required structure:
```
dwelling N5 → [path] → pick N4 → [path] → process N3 → [path] → drop N2 → [path] → dwelling
```

Look up node positions from `lif_file.json` (or `orderMessage_Example.json`) and
assign actions following the rules described in the code docstring.
The resulting order message sent to the robot must be identical to the one from Task 2.

**Part B — Implement `generate_order_message()`**
(`src/vda5050_interface/interfaces/order_interface.py`)

Implement the method so that it builds and publishes a VDA 5050-compliant order
message from whatever `nodes` and `edges` lists are passed in.

Input format (same for Task 3 manual path and Task 6 A* path):

```python
nodes = [
    {"nodeId": "N5", "x": 1.05, "y": 0.90, "theta": None, "actions": []},
    {"nodeId": "N12", "x": 4.00, "y": 3.50, "theta": None,
     "actions": [{"actionType": "init_fine_positioning",
                  "actionId": "...", "blockingType": "HARD"}]},
    {"nodeId": "N4", "x": 5.55, "y": 4.75, "theta": 0.0,
     "actions": [{"actionType": "pick", "actionId": "...", "blockingType": "HARD"}]},
    ...
]
edges = [
    {"edgeId": "E21", "startNodeId": "N5",  "endNodeId": "N12", "actions": []},
    {"edgeId": "E19", "startNodeId": "N12", "endNodeId": "N4",  "actions": []},
    ...
]
```

**Implementation steps for `generate_order_message()`:**
1. Build `nodes_msg`: for each entry in `nodes`, create a VDA 5050 node dict with
   `nodeId`, `sequenceId` (even, starting at 0), `released: true`,
   `nodePosition` (`x`, `y`, `mapId="Map_1"`, and `theta` only if not None),
   and `actions`.
2. Build `edges_msg`: for each entry in `edges`, create a VDA 5050 edge dict with
   `edgeId`, `sequenceId` (odd, starting at 1), `released: true`,
   `startNodeId`, `endNodeId`, and `actions`.
3. Assemble the full order message with all required top-level fields.
4. Publish via `self.mqtt_publisher.publish(order_msg, qos=0)`.
5. Increment `agent.agents.order_header_id` after publishing.

**Validation:** `pytest fleet_management/tests/test_task3_order_message.py -v`

---

### Task 4 — Implement the `Graph` Class

**File:** `src/fleet_management/graph.py`

Implement the four data-loading methods and two helper methods used by A*.

**Methods to implement:**

| Method | Returns |
|--------|---------|
| `get_nodes(lif_data)` | `{nodeId: {"nodeId": str, "pos": (x, y), ...}}` |
| `get_edges(lif_data)` | `{edgeId: {"edgeId": str, "startNodeId": str, "endNodeId": str, "startNodePos": (x,y), "endNodePos": (x,y)}}` |
| `get_stations(lif_data)` | `{stationId: {"interactionNodeIds": [...], "stationDescription": str}}` |
| `get_dwelling_nodes(lif_data)` | `[nodeId, ...]` |
| `get_connected_nodes(node_id)` | `[nodeId, ...]` (all neighbours) |
| `get_connected_edge(a, b)` | `edgeId` or `None` |

**Key hints:**
- All edges are **bidirectional**: `get_connected_nodes` and `get_connected_edge` must
  work regardless of which direction the edge is defined in the LIF.
- `get_stations` should only include `stationDescription == 'TRANSFER'` or `'PROCESS'`.
  Stations with `'CHARGING'` are dwelling nodes.
- Store `vehicleTypeNodeProperties` in `get_nodes` so that `fleet_management.py` can
  look up theta values for order messages.

**Validation:** `pytest fleet_management/tests/test_task4_graph.py -v`

---

### Task 5 — Extend the `Agent` Class

**File:** `src/fleet_management/agents.py`

Add the attributes needed by A* and task management to the `Agent` class,
and update `get_agents()` to initialize them from `agentsInitialization_file.json`.

**Attributes to add (minimum):**

| Attribute | Type | Description |
|-----------|------|-------------|
| `current_node` | `str` | Current node ID (used as A* start) |
| `current_task` | `dict` or `None` | Task dict from `task_management.task_list` |

**In `get_agents()`:** Loop over all entries in `agents_initialization_data['agents']`
(not just `[0]`) to support multiple agents.

**In `state_callback()`:** Update `current_node` from `state_msg['lastNodeId']`
(see Task 8 below).

**Validation:** `pytest fleet_management/tests/test_task5_agents.py -v`

---

### Task 6 — Implement A* Pathfinding

**File:** `src/fleet_management/fleet_management.py`

Implement the three `PathPlanning` methods in `fleet_management.py`.

**Methods to implement:**

| Method | Description |
|--------|-------------|
| `get_h(current, goal)` | Euclidean distance heuristic (use `math.dist`) |
| `get_distance(a, b)` | Euclidean distance between two adjacent nodes |
| `astar_search(start, goal)` | Returns `(path_nodes, path_edges)` or `(None, None)` |

`astar_search` must return:
- `path_nodes`: ordered list of node IDs, e.g. `["N5", "N1", "N7", "N2"]`
- `path_edges`: ordered list of edge IDs, e.g. `["E21", "E1", "E2"]`
  (`len(path_edges) == len(path_nodes) - 1`)

**Validation:** `pytest fleet_management/tests/test_task6_astar.py -v`

---

### Task 7 — Integrate A* into the Order Pipeline

**File:** `src/fleet_management/fleet_management.py`

Use the A* algorithm from Task 6 to automatically plan paths for multi-stop
transportation tasks and dispatch VDA 5050 order messages.

**Methods to implement in `FleetManagement`:**

| Method | Description |
|--------|-------------|
| `build_path_for_task(task, start_node)` | Chain A* calls for all stations + dwelling return |
| `build_order_nodes(path_nodes, task)` | Assign actions to each node in the path |
| `build_order_edges(path_nodes, path_edges)` | Build edges list for `generate_order_message()` |

**Action assignment rules for `build_order_nodes()`:**
- If a station has `actionType == 'pick'` or `'drop'` (TRANSFER):
  - Node **before** the station in `path_nodes` → `init_fine_positioning` action
  - Station node → `pick` or `drop` action
- If a station has `actionType == 'process'` (PROCESS):
  - Station node → `process` action with `processingTime` from the task

**Connect to `fleet_manager()`:**
Replace the manual path lists with the dynamic pipeline:
```python
task   = # first unassigned task
agent  = # first idle agent
path_nodes, path_edges = self.build_path_for_task(task, agent.current_node)
nodes  = self.build_order_nodes(path_nodes, task)
edges  = self.build_order_edges(path_nodes, path_edges)
agent.order_interface.generate_order_message(agent, orderId=..., order_updateId=0,
                                              nodes=nodes, edges=edges)
```

**Validation:** `pytest fleet_management/tests/test_task7_fleet_integration.py -v`

---

### Task 8 — Implement `state_callback()`

**File:** `src/fleet_management/agents.py`

Parse incoming VDA 5050 state messages and update the agent's attributes.

**Steps:**
1. Decode: `state_msg = json.loads(msg.payload.decode())`
2. Update `self.agvPosition` from `state_msg['agvPosition']`.
3. Update `self.current_node` from `state_msg['lastNodeId']`.
4. Detect task completion:
   - When `state_msg['nodeStates'] == []` AND `state_msg['edgeStates'] == []`
     AND all actions in `state_msg['actionStates']` have `actionStatus == 'FINISHED'`,
     the robot has finished its current order.
   - Set the current task's `'task_completed' = True`.
   - Set `self.agent_state = 'IDLE'`.

**Validation:** `pytest fleet_management/tests/test_task8_state_callback.py -v`

---

### Task 9 — Implement `TaskAssignment`

**File:** `src/fleet_management/task_assignment.py`

Implement `task_assignment_manager()` so that idle agents automatically receive
the next unassigned task.

**Steps:**
1. Loop until all tasks in `self.task_management.task_list` are assigned.
2. Find idle agents (`agent_state == "IDLE"`) and unassigned tasks.
3. For each idle-agent/task pair:
   - `task['task_assigned'] = True`
   - `task['agent_id'] = agent.agentId`
   - `agent.agent_state = "EXECUTING"`
   - `agent.current_task = task`
4. (Optional) Prefer the idle agent **closest** to the task's first station node.
5. Sleep briefly (`time.sleep(0.5)`) between iterations.

Start the manager in a daemon thread (uncomment the line in `__init__`).

**Validation:** `pytest fleet_management/tests/test_task9_task_assignment.py -v`

---

### Task 10 — Full Automation

**Files:** `src/fleet_management/fleet_management.py`,
`src/fleet_management/task_assignment.py`

Combine Tasks 7, 8, and 9 to execute all tasks automatically end-to-end.

**In `fleet_manager()`:**
- Replace the single-shot call with a while loop:
  ```python
  while any(not t['task_assigned'] for t in self.task_management.task_list):
      # wait for an idle agent
      # pick the next assigned-but-not-yet-ordered task
      # plan path and send order
      time.sleep(0.5)
  ```
- Run `fleet_manager()` in a daemon thread from `__init__()`.

**Validation:** `pytest fleet_management/tests/test_task10_fleet_manager.py -v`

**Benchmark:** All 5 transportation tasks should complete automatically.

---

## Testing

Run all tests from the **workspace root** (`imrl_workspace/`) directory.

### Run individual task tests

```bash
# Task 3 — generate_order_message()
pytest fleet_management/tests/test_task3_order_message.py -v

# Task 4 — Graph class
pytest fleet_management/tests/test_task4_graph.py -v

# Task 5 — Agent digital twin
pytest fleet_management/tests/test_task5_agents.py -v

# Task 6 — A* pathfinding (PathPlanning)
pytest fleet_management/tests/test_task6_astar.py -v

# Task 7 — Fleet management integration
pytest fleet_management/tests/test_task7_fleet_integration.py -v

# Task 8 — state_callback()
pytest fleet_management/tests/test_task8_state_callback.py -v

# Task 9 — TaskAssignment
pytest fleet_management/tests/test_task9_task_assignment.py -v

# Task 10 — Full automation (fleet_manager loop + daemon thread)
pytest fleet_management/tests/test_task10_fleet_manager.py -v
```

### Run all tests at once

```bash
pytest fleet_management/ -v
```

### Notes

- Tests are independent per file — each tests exactly one task.
- No MQTT broker is needed — broker calls are mocked in all tests.
- The simulation executable does **not** need to be running.


---

## Important Notes

### VDA 5050 Protocol

- **Order message:** Sent by the fleet management to command the robot.
  Nodes have **even** sequenceIds (0, 2, 4, …), edges have **odd** sequenceIds (1, 3, 5, …).
- **State message:** Sent by the robot to report its current status.
  `nodeStates` contains remaining nodes, `edgeStates` remaining edges.
- `orderId` is a string and must be unique per order.
- `released: true` marks nodes/edges that the robot is allowed to traverse.

### MQTT Topics

| Topic | Direction | Content |
|-------|-----------|---------|
| `KIT/IMRL/mouse001/order` | Fleet → Robot | VDA 5050 order message |
| `KIT/IMRL/mouse001/state` | Robot → Fleet | VDA 5050 state message |
| `KIT/IMRL/tasks` | Fleet → Visualizer | Task list (for visualization) |

### Threading (Tasks 9 & 10)

Use Python's `threading.Thread` with `daemon=True` so that background threads
terminate automatically when the main process exits.
Always use `time.sleep()` inside loops to avoid busy-waiting.

### Common Pitfalls

- **Path chaining:** When concatenating A* legs, skip the duplicate junction node
  (last node of leg N = first node of leg N+1).
- **init_fine_positioning:** Must appear on the node **before** the TRANSFER station,
  not on the station itself.
- **theta as string:** In `lif_file.json`, theta is sometimes stored as the string `"None"`.
  Check for this and convert to Python `None` before using it.
- **Simulation startup time:** The executable may take longer than 5 seconds to start
  on some systems. Increase `time.sleep(5)` in `run_fleet_management_simulation.py` if needed.
- **Permission error:** If you receive `PermissionError: [Error 13] Permission denied: 'src/mobile_robot_simulation/dist/agent_simulation'`, navigate to `fleet_management/src/mobile_robot_simulation/dist/` and run:
  ```bash
  chmod +x agent_simulation
  ```
