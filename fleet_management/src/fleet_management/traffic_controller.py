
import copy
import time
import threading


class TrafficController:
    """
    Dynamic Zone Control using VDA 5050 OrderUpdates.

    Normal behaviour:
    - Release only a short safe horizon.
    - Higher-priority AGV proceeds.
    - Lower-priority AGV waits.

    Rare deadlock fallback:
    - If the same lower-priority AGV remains blocked for several cycles,
      insert a temporary route:
          current node -> safe neighbour -> current node -> original route
    - The AGV waits at the safe neighbour until the higher-priority route
      clears the contested node.
    """

    CONTROL_PERIOD_SECONDS = 0.5
    DEADLOCK_TRIGGER_CYCLES = 10  # 10 × 0.5 seconds = 5 seconds

    def __init__(self, fleet_manager):
        self.fleet_manager = fleet_manager
        self.running = True

        # Key:
        # (agent_id, current_node, contested_node, blocking_agent_id)
        self.deadlock_counts = {}

        self.thread = threading.Thread(
            target=self.loop,
            daemon=True,
        )
        self.thread.start()

    # ------------------------------------------------------------------
    # Priority
    # ------------------------------------------------------------------

    @staticmethod
    def get_priority(agent):
        """
        Lower tuple = higher priority.

        Priority is based on:
        - remaining route nodes;
        - remaining actions;
        - agent ID as deterministic tie-breaker.
        """
        if (
            not hasattr(agent, "full_nodes")
            or not hasattr(agent, "released_index")
        ):
            return float("inf"), agent.agentId

        remaining_nodes = max(
            0,
            len(agent.full_nodes) - agent.released_index,
        )

        remaining_actions = 0

        for index in range(
            agent.released_index,
            len(agent.full_nodes),
        ):
            remaining_actions += len(
                agent.full_nodes[index].get("actions", [])
            )

        score = remaining_nodes + remaining_actions * 3

        return score, agent.agentId

    @staticmethod
    def is_lower_priority(agent, other):
        """
        Return True when agent must yield to other.
        """
        return TrafficController.get_priority(
            agent
        ) > TrafficController.get_priority(other)

    # ------------------------------------------------------------------
    # Graph helpers
    # ------------------------------------------------------------------

    def get_node_theta(self, node_id, vehicle_type_id):
        """
        Read vehicle-specific orientation for a graph node.
        """
        node_data = self.fleet_manager.graph.nodes[node_id]

        for prop in node_data.get(
            "vehicleTypeNodeProperties",
            [],
        ):
            if prop.get("vehicleTypeId") != vehicle_type_id:
                continue

            theta = prop.get("theta")

            if theta is None or theta == "None":
                return None

            return float(theta)

        return None

    def build_transit_node(self, node_id, vehicle_type_id):
        """
        Create a node dictionary without station actions.

        Escape nodes and return nodes must not execute pick, drop,
        process, or fine-positioning actions.
        """
        node_data = self.fleet_manager.graph.nodes[node_id]
        x, y = node_data["pos"]

        return {
            "nodeId": node_id,
            "x": x,
            "y": y,
            "theta": self.get_node_theta(
                node_id,
                vehicle_type_id,
            ),
            "actions": [],
            "released": False,
        }

    def build_transit_edge(
        self,
        start_node,
        end_node,
        vehicle_type_id,
    ):
        """
        Build one permitted edge dictionary.
        """
        graph = self.fleet_manager.graph

        edge_id = graph.get_connected_edge(
            start_node,
            end_node,
            vehicle_type_id,
        )

        if edge_id is None:
            return None

        edge_data = graph.edges[edge_id]

        result = {
            "edgeId": edge_id,
            "startNodeId": start_node,
            "endNodeId": end_node,
            "actions": [],
            "released": False,
        }

        if edge_data.get("trajectory") is not None:
            result["trajectory"] = edge_data["trajectory"]

        return result

    def get_station_nodes(self):
        """
        Return all transfer/process interaction nodes.
        """
        station_nodes = set()

        for station in self.fleet_manager.graph.stations.values():
            station_nodes.update(
                station.get("interactionNodeIds", [])
            )

        return station_nodes

    def find_safe_escape_node(
        self,
        yielding_agent,
        priority_agent,
        contested_node,
        agent_physical,
        agent_intent,
    ):
        """
        Find a directly connected safe neighbour for the yielding AGV.

        Rejected candidates:
        - contested node;
        - occupied node;
        - node in higher-priority AGV's local route;
        - node in another AGV's local route;
        - edge forbidden for this vehicle type.

        Ordinary transit nodes are preferred over station nodes.
        """
        graph = self.fleet_manager.graph

        current_node = getattr(
            yielding_agent,
            "current_node",
            None,
        )

        if current_node is None:
            return None

        vehicle_type_id = getattr(
            yielding_agent,
            "vehicle_type_id",
            None,
        )

        neighbours = graph.get_connected_nodes(
            current_node,
            vehicle_type_id,
        )

        occupied_nodes = set()

        for physical_nodes in agent_physical.values():
            occupied_nodes.update(physical_nodes)

        priority_path = set(
            getattr(priority_agent, "tracked_path", [])
        )
        priority_path.update(
            agent_intent.get(priority_agent, set())
        )

        all_other_intents = set()

        for other, intended_nodes in agent_intent.items():
            if other != yielding_agent:
                all_other_intents.update(intended_nodes)

        station_nodes = self.get_station_nodes()

        ordinary_candidates = []
        station_candidates = []

        for neighbour in neighbours:
            if neighbour == contested_node:
                continue

            if neighbour in occupied_nodes:
                continue

            if neighbour in priority_path:
                continue

            if neighbour in all_other_intents:
                continue

            edge = self.build_transit_edge(
                current_node,
                neighbour,
                vehicle_type_id,
            )

            reverse_edge = self.build_transit_edge(
                neighbour,
                current_node,
                vehicle_type_id,
            )

            # The temporary manoeuvre must permit travel out and back.
            if edge is None or reverse_edge is None:
                continue

            if neighbour in station_nodes:
                station_candidates.append(neighbour)
            else:
                ordinary_candidates.append(neighbour)

        candidates = ordinary_candidates or station_candidates

        if not candidates:
            return None

        # Deterministic result for repeatable simulations.
        return sorted(candidates)[0]

    # ------------------------------------------------------------------
    # Escape-route handling
    # ------------------------------------------------------------------

    def install_escape_route(
        self,
        yielding_agent,
        priority_agent,
        contested_node,
        escape_node,
    ):
        """
        Insert:

            current -> escape -> current -> original remaining route

        The current node must be the final previously released node.
        Therefore the temporary route remains a valid VDA 5050 extension.
        """
        current_node = getattr(
            yielding_agent,
            "current_node",
            None,
        )

        current_idx = getattr(
            yielding_agent,
            "tracked_current_idx",
            0,
        )

        released_index = getattr(
            yielding_agent,
            "released_index",
            0,
        )

        if current_node is None:
            return False

        if not (
            0 <= current_idx < len(yielding_agent.full_nodes)
        ):
            return False

        if (
            yielding_agent.full_nodes[current_idx].get("nodeId")
            != current_node
        ):
            return False

        # Safe rerouting is possible only before any node beyond the current
        # node has been released.
        if released_index != current_idx + 1:
            print(
                f"[TrafficController] Cannot reroute "
                f"{yielding_agent.agentId}: released_index="
                f"{released_index}, expected {current_idx + 1}"
            )
            return False

        vehicle_type_id = getattr(
            yielding_agent,
            "vehicle_type_id",
            None,
        )

        edge_to_escape = self.build_transit_edge(
            current_node,
            escape_node,
            vehicle_type_id,
        )

        edge_back = self.build_transit_edge(
            escape_node,
            current_node,
            vehicle_type_id,
        )

        if edge_to_escape is None or edge_back is None:
            return False

        escape_node_data = self.build_transit_node(
            escape_node,
            vehicle_type_id,
        )

        return_node_data = self.build_transit_node(
            current_node,
            vehicle_type_id,
        )

        old_nodes = yielding_agent.full_nodes
        old_edges = yielding_agent.full_edges

        # Keep all completed/current nodes.
        prefix_nodes = old_nodes[: current_idx + 1]

        # Keep the original route after the current node.
        suffix_nodes = old_nodes[current_idx + 1 :]

        # Edges before the current node remain unchanged.
        prefix_edges = old_edges[:current_idx]

        # The original outgoing edge and all later edges remain unchanged.
        suffix_edges = old_edges[current_idx:]

        yielding_agent.full_nodes = (
            prefix_nodes
            + [escape_node_data, return_node_data]
            + suffix_nodes
        )

        yielding_agent.full_edges = (
            prefix_edges
            + [edge_to_escape, edge_back]
            + suffix_edges
        )

        # Preserve historical release state only through the current node.
        for index, node in enumerate(
            yielding_agent.full_nodes
        ):
            node["released"] = index <= current_idx

        for index, edge in enumerate(
            yielding_agent.full_edges
        ):
            edge["released"] = index < current_idx

        # Release the escape movement immediately.
        yielding_agent.full_nodes[
            current_idx + 1
        ]["released"] = True

        yielding_agent.full_edges[
            current_idx
        ]["released"] = True

        yielding_agent.released_index = current_idx + 2

        yielding_agent.yield_mode = {
            "phase": "MOVING_TO_ESCAPE",
            "escape_node": escape_node,
            "return_node": current_node,
            "contested_node": contested_node,
            "priority_agent_id": priority_agent.agentId,
        }

        print(
            f"[TrafficController] DEADLOCK FALLBACK: "
            f"{yielding_agent.agentId} yields to "
            f"{priority_agent.agentId}; moving "
            f"{current_node} -> {escape_node}"
        )

        return True

    def find_agent_by_id(self, agent_id):
        for agent in self.fleet_manager.agents.agents:
            if agent.agentId == agent_id:
                return agent

        return None

    def priority_route_has_cleared(
        self,
        yielding_agent,
        agent_physical,
        agent_intent,
    ):
        """
        Check whether the higher-priority AGV has cleared the contested node.
        """
        yield_mode = getattr(
            yielding_agent,
            "yield_mode",
            None,
        )

        if not yield_mode:
            return True

        priority_agent = self.find_agent_by_id(
            yield_mode["priority_agent_id"]
        )

        if priority_agent is None:
            return True

        contested_node = yield_mode["contested_node"]

        # The yielding robot must never return while the priority robot is
        # physically standing on the contested node.
        if contested_node in agent_physical.get(
            priority_agent,
            set(),
        ):
            return False

        priority_actions = getattr(
            priority_agent,
            "actionStates",
            [],
        )

        # Once all task actions of the priority robot are complete, allow the
        # yielding robot to return and clear its temporary escape node. The
        # priority robot may still have a dwelling/parking route containing
        # the contested node, but that must not keep the yielding robot parked
        # forever.
        if priority_actions and all(
            action.get("actionStatus") == "FINISHED"
            for action in priority_actions
        ):
            return True

        # While the priority robot still has unfinished task actions, continue
        # protecting its near-term route.
        if contested_node in agent_intent.get(
            priority_agent,
            set(),
        ):
            return False

        return True

    def process_yield_mode(
        self,
        agent,
        agent_physical,
        agent_intent,
    ):
        """
        Control the temporary yielding route.

        Returns:
            True  -> skip normal release processing this cycle
            False -> continue normal processing
        """
        yield_mode = getattr(agent, "yield_mode", None)

        if not yield_mode:
            return False

        current_node = getattr(agent, "current_node", None)
        escape_node = yield_mode["escape_node"]
        return_node = yield_mode["return_node"]

        if (
            yield_mode["phase"] == "MOVING_TO_ESCAPE"
            and current_node == escape_node
        ):
            yield_mode["phase"] = "WAITING_AT_ESCAPE"

            print(
                f"[TrafficController] {agent.agentId} reached "
                f"escape node {escape_node}; waiting for "
                f"{yield_mode['priority_agent_id']}"
            )

        if yield_mode["phase"] == "WAITING_AT_ESCAPE":
            if not self.priority_route_has_cleared(
                agent,
                agent_physical,
                agent_intent,
            ):
                return True

            current_idx = getattr(
                agent,
                "tracked_current_idx",
                0,
            )

            next_idx = current_idx + 1

            if next_idx >= len(agent.full_nodes):
                return True

            if (
                agent.full_nodes[next_idx].get("nodeId")
                != return_node
            ):
                print(
                    f"[TrafficController] Invalid escape return "
                    f"route for {agent.agentId}"
                )
                return True

            agent.full_nodes[next_idx]["released"] = True

            if current_idx < len(agent.full_edges):
                agent.full_edges[current_idx]["released"] = True

            agent.released_index = next_idx + 1
            agent.order_update_id += 1

            agent.order_interface.generate_order_message(
                agent=agent,
                orderId=agent.current_order_id,
                order_updateId=agent.order_update_id,
                nodes=agent.full_nodes[current_idx:],
                edges=agent.full_edges[current_idx:],
                start_sequence_idx=current_idx,
            )

            yield_mode["phase"] = "RETURNING"

            print(
                f"[TrafficController] {agent.agentId} returning "
                f"{escape_node} -> {return_node}"
            )

            return True

        if (
            yield_mode["phase"] == "RETURNING"
            and current_node == return_node
        ):
            print(
                f"[TrafficController] {agent.agentId} completed "
                f"yield manoeuvre and resumes its original route"
            )

            delattr(agent, "yield_mode")

            # Continue normal release processing from the return node.
            return False

        # While travelling to the escape or return node, do not release
        # additional nodes.
        return True

    # ------------------------------------------------------------------
    # Main controller
    # ------------------------------------------------------------------

    def loop(self):
        while self.running:
            agents = self.fleet_manager.agents.agents

            agent_physical = {
                agent: set()
                for agent in agents
            }

            agent_intent = {
                agent: set()
                for agent in agents
            }

            # ----------------------------------------------------------
            # Build physical and local-intent registries
            # ----------------------------------------------------------

            for agent in agents:
                current_node = getattr(
                    agent,
                    "current_node",
                    None,
                )

                if current_node:
                    agent_physical[agent].add(current_node)

                if (
                    agent.agent_state != "EXECUTING"
                    or not hasattr(agent, "full_nodes")
                    or not agent.full_nodes
                ):
                    continue

                current_idx = getattr(
                    agent,
                    "tracked_current_idx",
                    0,
                )

                current_idx = min(
                    max(current_idx, 0),
                    len(agent.full_nodes) - 1,
                )

                # Search forward first because routes can contain the same
                # node multiple times.
                found_idx = None

                if current_node:
                    for index in range(
                        current_idx,
                        len(agent.full_nodes),
                    ):
                        if (
                            agent.full_nodes[index].get("nodeId")
                            == current_node
                        ):
                            found_idx = index
                            break

                    if found_idx is None:
                        for index in range(current_idx):
                            if (
                                agent.full_nodes[index].get("nodeId")
                                == current_node
                            ):
                                found_idx = index
                                break

                if found_idx is not None:
                    current_idx = found_idx

                agent.tracked_current_idx = current_idx

                # Local route intent only.
                intent_horizon = 6
                horizon_end = min(
                    current_idx + intent_horizon,
                    len(agent.full_nodes),
                )

                path_slice = []

                for index in range(
                    current_idx,
                    horizon_end,
                ):
                    node_id = agent.full_nodes[index]["nodeId"]

                    if node_id not in path_slice:
                        path_slice.append(node_id)

                    agent_intent[agent].add(node_id)

                agent.tracked_path = path_slice

            # ----------------------------------------------------------
            # Process higher-priority AGVs first
            # ----------------------------------------------------------

            ordered_agents = sorted(
                agents,
                key=self.get_priority,
            )

            any_blocked = False

            for agent in ordered_agents:
                if (
                    agent.agent_state != "EXECUTING"
                    or not hasattr(agent, "full_nodes")
                    or not agent.full_nodes
                ):
                    continue

                # Avoid sending OrderUpdate 1 before the initial order.
                if not getattr(
                    agent,
                    "initial_order_sent",
                    True,
                ):
                    continue

                # Preserve the N1A working behaviour:
                # never send an update while a station action is RUNNING.
                has_running_action = any(
                    action.get("actionStatus") == "RUNNING"
                    for action in getattr(
                        agent,
                        "actionStates",
                        [],
                    )
                )

                if has_running_action:
                    continue

                if self.process_yield_mode(
                    agent,
                    agent_physical,
                    agent_intent,
                ):
                    continue

                updated = False
                escape_update_created = False

                update_anchor_idx = max(
                    0,
                    getattr(agent, "released_index", 0) - 1,
                )

                while (
                    getattr(agent, "released_index", 0)
                    < len(agent.full_nodes)
                ):
                    current_idx = getattr(
                        agent,
                        "tracked_current_idx",
                        0,
                    )

                    # Release no more than one node ahead.
                    #
                    # This is necessary because a released VDA 5050 route
                    # cannot safely be revoked for an escape manoeuvre.
                    if (
                        agent.released_index - current_idx
                        >= 2
                    ):
                        break

                    next_node_id = agent.full_nodes[
                        agent.released_index
                    ]["nodeId"]

                    is_blocked = False
                    blocking_agent = None

                    for other in agents:
                        if other == agent:
                            continue

                        # Rule A: actual physical occupancy.
                        if next_node_id in agent_physical[other]:
                            is_blocked = True
                            blocking_agent = other

                            print(
                                f"[TrafficController] "
                                f"{agent.agentId} blocked from "
                                f"{next_node_id}: physically occupied "
                                f"by {other.agentId}"
                            )
                            break

                        other_intent = agent_intent.get(
                            other,
                            set(),
                        )

                        if next_node_id not in other_intent:
                            continue

                        overlap = (
                            agent_intent[agent]
                            .intersection(other_intent)
                        )

                        agent_overlap = [
                            node
                            for node in getattr(
                                agent,
                                "tracked_path",
                                [],
                            )
                            if node in overlap
                        ]

                        other_overlap = [
                            node
                            for node in getattr(
                                other,
                                "tracked_path",
                                [],
                            )
                            if node in overlap
                        ]

                        is_head_on = False

                        if (
                            len(agent_overlap) >= 2
                            and len(other_overlap) >= 2
                        ):
                            first = agent_overlap[0]
                            second = agent_overlap[1]

                            if (
                                first in other_overlap
                                and second in other_overlap
                                and other_overlap.index(first)
                                > other_overlap.index(second)
                            ):
                                is_head_on = True

                        my_current = getattr(
                            agent,
                            "current_node",
                            None,
                        )

                        other_current = getattr(
                            other,
                            "current_node",
                            None,
                        )

                        my_inside = my_current in overlap
                        other_inside = other_current in overlap

                        if is_head_on:
                            if other_inside and not my_inside:
                                is_blocked = True
                                blocking_agent = other

                            elif my_inside and not other_inside:
                                # Agent already inside: let it clear.
                                is_blocked = False

                            elif self.is_lower_priority(
                                agent,
                                other,
                            ):
                                is_blocked = True
                                blocking_agent = other

                        else:
                            if self.is_lower_priority(
                                agent,
                                other,
                            ):
                                is_blocked = True
                                blocking_agent = other

                        if is_blocked:
                            print(
                                f"[TrafficController] "
                                f"{agent.agentId} yielding before "
                                f"{next_node_id} to "
                                f"{other.agentId}; overlap={overlap}"
                            )
                            break

                    if is_blocked:
                        any_blocked = True

                        if (
                            blocking_agent is not None
                            and self.is_lower_priority(
                                agent,
                                blocking_agent,
                            )
                        ):
                            conflict_key = (
                                agent.agentId,
                                getattr(
                                    agent,
                                    "current_node",
                                    None,
                                ),
                                next_node_id,
                                blocking_agent.agentId,
                            )

                            self.deadlock_counts[
                                conflict_key
                            ] = (
                                self.deadlock_counts.get(
                                    conflict_key,
                                    0,
                                )
                                + 1
                            )

                            cycles = self.deadlock_counts[
                                conflict_key
                            ]

                            print(
                                f"[TrafficController] Conflict "
                                f"{conflict_key}: "
                                f"{cycles}/"
                                f"{self.DEADLOCK_TRIGGER_CYCLES}"
                            )

                            if (
                                cycles
                                >= self.DEADLOCK_TRIGGER_CYCLES
                            ):
                                escape_node = (
                                    self.find_safe_escape_node(
                                        yielding_agent=agent,
                                        priority_agent=blocking_agent,
                                        contested_node=next_node_id,
                                        agent_physical=agent_physical,
                                        agent_intent=agent_intent,
                                    )
                                )

                                if escape_node is None:
                                    print(
                                        f"[TrafficController] No safe "
                                        f"escape neighbour found for "
                                        f"{agent.agentId}"
                                    )

                                else:
                                    installed = (
                                        self.install_escape_route(
                                            yielding_agent=agent,
                                            priority_agent=blocking_agent,
                                            contested_node=next_node_id,
                                            escape_node=escape_node,
                                        )
                                    )

                                    if installed:
                                        agent.order_update_id += 1

                                        anchor = (
                                            agent.tracked_current_idx
                                        )

                                        agent.order_interface.generate_order_message(
                                            agent=agent,
                                            orderId=agent.current_order_id,
                                            order_updateId=agent.order_update_id,
                                            nodes=agent.full_nodes[anchor:],
                                            edges=agent.full_edges[anchor:],
                                            start_sequence_idx=anchor,
                                        )

                                        escape_update_created = True

                                        # Remove all stale counters for
                                        # this yielding AGV.
                                        self.deadlock_counts = {
                                            key: value
                                            for key, value
                                            in self.deadlock_counts.items()
                                            if key[0] != agent.agentId
                                        }

                        break

                    # Conflict cleared: remove stale counters for this agent
                    # and this next node.
                    self.deadlock_counts = {
                        key: value
                        for key, value
                        in self.deadlock_counts.items()
                        if not (
                            key[0] == agent.agentId
                            and key[2] == next_node_id
                        )
                    }

                    # Release one safe node.
                    agent.full_nodes[
                        agent.released_index
                    ]["released"] = True

                    edge_index = agent.released_index - 1

                    if (
                        0
                        <= edge_index
                        < len(agent.full_edges)
                    ):
                        agent.full_edges[
                            edge_index
                        ]["released"] = True

                    agent.released_index += 1
                    updated = True

                if escape_update_created:
                    continue

                if updated:
                    agent.order_update_id += 1

                    self.fleet_manager.agents.logging.info(
                        f"[TrafficController] Sending "
                        f"OrderUpdate {agent.order_update_id} "
                        f"to {agent.agentId}, releasing up to "
                        f"index {agent.released_index}"
                    )

                    start_idx = min(
                        update_anchor_idx,
                        len(agent.full_nodes) - 1,
                    )

                    continuation_nodes = (
                        agent.full_nodes[start_idx:]
                    )

                    continuation_edges = (
                        agent.full_edges[start_idx:]
                    )

                    agent.order_interface.generate_order_message(
                        agent=agent,
                        orderId=getattr(
                            agent,
                            "current_order_id",
                            str(
                                self.fleet_manager
                                .agents
                                .order_header_id
                            ),
                        ),
                        order_updateId=agent.order_update_id,
                        nodes=continuation_nodes,
                        edges=continuation_edges,
                        start_sequence_idx=start_idx,
                    )

            if any_blocked:
                for agent in agents:
                    statuses = [
                        action.get("actionStatus")
                        for action in getattr(
                            agent,
                            "actionStates",
                            [],
                        )
                    ]

                    print(
                        f"[STATUS-{agent.agentId}] "
                        f"state={agent.agent_state}, "
                        f"curr_node={getattr(agent, 'current_node', None)}, "
                        f"released_idx="
                        f"{getattr(agent, 'released_index', None)}, "
                        f"yield_mode="
                        f"{getattr(agent, 'yield_mode', None)}, "
                        f"actions={statuses}"
                    )

            time.sleep(self.CONTROL_PERIOD_SECONDS)
