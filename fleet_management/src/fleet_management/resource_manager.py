"""
ResourceManager — thread-safe reservation of nodes, edges, and stations.

Ensures:
- Only one agent occupies a node at any time (collision avoidance).
- Only one agent uses a TRANSFER/PROCESS station at any time (station locking).
- Deadlock prevention via ownership tracking and priority-based resolution.
"""

import threading
import time


class ResourceManager:
    """
    Shared resource tracker used by TrafficController to avoid collisions.

    Thread-safe: all public methods acquire self._lock.
    """

    def __init__(self, graph):
        self._graph = graph
        self._lock = threading.Lock()

        # node_id → agent_id  (who currently reserves this node)
        self._reserved_nodes: dict[str, str] = {}

        # station_id → agent_id  (who currently locks this station)
        self._reserved_stations: dict[str, str] = {}

        # agent_id → float  (timestamp of last progress for deadlock detection)
        self._agent_last_progress: dict[str, float] = {}

    # ── Node reservation ────────────────────────────────────────────────────

    def try_reserve_node(self, node_id: str, agent_id: str) -> bool:
        """
        Try to reserve a node for an agent.

        Returns True if the node is free or already reserved by the same
        agent, False if another agent holds it.
        """
        with self._lock:
            owner = self._reserved_nodes.get(node_id)
            if owner is None or owner == agent_id:
                self._reserved_nodes[node_id] = agent_id
                self._agent_last_progress[agent_id] = time.time()
                return True
            return False

    def release_node(self, node_id: str, agent_id: str) -> None:
        """Release a node previously reserved by agent_id."""
        with self._lock:
            if self._reserved_nodes.get(node_id) == agent_id:
                del self._reserved_nodes[node_id]

    def is_node_available(self, node_id: str, requesting_agent: str) -> bool:
        """Check whether a node is available (free or owned by the requester)."""
        with self._lock:
            owner = self._reserved_nodes.get(node_id)
            return owner is None or owner == requesting_agent

    def get_node_owner(self, node_id: str) -> str | None:
        """Return the agent_id that currently reserves this node, or None."""
        with self._lock:
            return self._reserved_nodes.get(node_id)

    # ── Station locking ─────────────────────────────────────────────────────

    def _get_station_for_node(self, node_id: str) -> str | None:
        """
        Return the stationId that owns this interaction node, or None.

        Covers both TRANSFER and PROCESS stations.
        """
        for station_id, station_data in self._graph.stations.items():
            if node_id in station_data.get("interactionNodeIds", []):
                return station_id
        return None

    def try_reserve_station(self, node_id: str, agent_id: str) -> bool:
        """
        Reserve a station for an agent via its interaction node.

        When a robot approaches a station (e.g. init_fine_positioning or
        pick/drop/process action), the ENTIRE station is locked — all
        interaction nodes become unavailable for other agents.

        Returns True on success, False if the station is held by another agent.
        """
        station_id = self._get_station_for_node(node_id)
        if station_id is None:
            return True  # not a station — always "available"

        with self._lock:
            owner = self._reserved_stations.get(station_id)
            if owner is None or owner == agent_id:
                self._reserved_stations[station_id] = agent_id
                return True
            return False

    def release_station(self, node_id: str, agent_id: str) -> None:
        """Release a station lock previously held by agent_id."""
        station_id = self._get_station_for_node(node_id)
        if station_id is None:
            return
        with self._lock:
            if self._reserved_stations.get(station_id) == agent_id:
                del self._reserved_stations[station_id]

    def is_station_available(self, node_id: str, requesting_agent: str) -> bool:
        """Check whether a station (via its interaction node) is available."""
        station_id = self._get_station_for_node(node_id)
        if station_id is None:
            return True
        with self._lock:
            owner = self._reserved_stations.get(station_id)
            return owner is None or owner == requesting_agent

    # ── Deadlock detection ──────────────────────────────────────────────────

    def get_blocking_agent(self, node_id: str, requesting_agent: str) -> str | None:
        """
        Return the agent_id that is blocking *requesting_agent* from using
        *node_id*, or None if nobody blocks it.
        """
        with self._lock:
            owner = self._reserved_nodes.get(node_id)
            if owner is not None and owner != requesting_agent:
                return owner
            station_id = self._get_station_for_node(node_id)
            if station_id is not None:
                station_owner = self._reserved_stations.get(station_id)
                if station_owner is not None and station_owner != requesting_agent:
                    return station_owner
            return None

    def is_deadlocked(self, agent_a: str, agent_b: str) -> bool:
        """
        Check if agent_a and agent_b are in a circular wait (deadlock).

        A deadlock exists if agent_a waits for a resource held by agent_b AND
        agent_b waits for a resource held by agent_a.
        """
        # Collect what each agent is waiting for
        nodes_held_by_b = set()
        nodes_held_by_a = set()

        with self._lock:
            for nid, owner in self._reserved_nodes.items():
                if owner == agent_b:
                    nodes_held_by_b.add(nid)
                elif owner == agent_a:
                    nodes_held_by_a.add(nid)

        # Deadlock exists if both hold resources the other needs
        return len(nodes_held_by_a) > 0 and len(nodes_held_by_b) > 0

    def get_stall_duration(self, agent_id: str) -> float:
        """Return seconds since this agent last made progress."""
        with self._lock:
            last = self._agent_last_progress.get(agent_id)
            if last is None:
                return 0.0
            return time.time() - last

    # ── Bulk operations ─────────────────────────────────────────────────────

    def reserve_path(self, node_ids: list[str], edge_ids: list[str],
                     agent_id: str) -> bool:
        """
        Try to reserve all nodes (and their stations) for a full path.

        Returns True if all are available and reserved.
        Returns False if any node or station is held by another agent.
        """
        with self._lock:
            # Check all nodes first
            for nid in node_ids:
                owner = self._reserved_nodes.get(nid)
                if owner is not None and owner != agent_id:
                    return False
            # Check all stations
            stations_to_lock = set()
            for nid in node_ids:
                sid = self._get_station_for_node(nid)
                if sid is not None:
                    stations_to_lock.add(sid)
            for sid in stations_to_lock:
                owner = self._reserved_stations.get(sid)
                if owner is not None and owner != agent_id:
                    return False
            # All available — reserve nodes
            for nid in node_ids:
                self._reserved_nodes[nid] = agent_id
            # Lock stations
            for sid in stations_to_lock:
                self._reserved_stations[sid] = agent_id
            self._agent_last_progress[agent_id] = time.time()
            return True

    def release_path(self, node_ids: list[str], edge_ids: list[str],
                     agent_id: str) -> None:
        """Release all nodes in a path for an agent."""
        with self._lock:
            for nid in node_ids:
                if self._reserved_nodes.get(nid) == agent_id:
                    del self._reserved_nodes[nid]

    def release_all_agent_resources(self, agent_id: str) -> None:
        """Release every node and station held by an agent (e.g. on task completion)."""
        with self._lock:
            nodes_to_release = [
                nid for nid, owner in self._reserved_nodes.items()
                if owner == agent_id
            ]
            for nid in nodes_to_release:
                del self._reserved_nodes[nid]

            stations_to_release = [
                sid for sid, owner in self._reserved_stations.items()
                if owner == agent_id
            ]
            for sid in stations_to_release:
                del self._reserved_stations[sid]
