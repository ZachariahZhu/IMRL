class Graph:
    """
    Class representing the layout as a graph.
    Nodes, edges, stations, and dwelling nodes are read from the LIF file.

    Task 4: Model the layout as a graph.
    Task 2c: Add vehicle-type-aware edge filtering.
    """

    def __init__(self, lif_data):
        self.nodes = self.get_nodes(lif_data)
        self.edges = self.get_edges(lif_data)
        self.stations = self.get_stations(lif_data)
        self.dwelling_nodes = self.get_dwelling_nodes(lif_data)

    # ── Task 4 + Task 2c ─────────────────────────────────────────────────────

    def get_nodes(self, lif_data) -> dict:
        """
        Read all nodes from the LIF file.
        """
        nodes = {}

        for n in lif_data["layouts"][0]["nodes"]:
            node_id = n["nodeId"]

            nodes[node_id] = {
                "nodeId": node_id,
                "pos": (
                    n["nodePosition"]["x"],
                    n["nodePosition"]["y"]
                ),
                "vehicleTypeNodeProperties": n.get(
                    "vehicleTypeNodeProperties",
                    []
                )
            }

        return nodes

    def get_edges(self, lif_data) -> dict:
        """
        Read all edges from the LIF file.

        Task 2c addition:
        Each edge also stores allowedVehicleTypes.
        """
        edges = {}

        for e in lif_data["layouts"][0]["edges"]:
            edge_id = e["edgeId"]
            start_id = e["startNodeId"]
            end_id = e["endNodeId"]

            allowed_vehicle_types = []

            if "vehicleTypeEdgeProperties" in e:
                for prop in e["vehicleTypeEdgeProperties"]:
                    vehicle_type_id = prop.get("vehicleTypeId")

                    if vehicle_type_id is not None:
                        allowed_vehicle_types.append(vehicle_type_id)

            edges[edge_id] = {
                "edgeId": edge_id,
                "startNodeId": start_id,
                "endNodeId": end_id,
                "startNodePos": self.nodes[start_id]["pos"],
                "endNodePos": self.nodes[end_id]["pos"],
                "allowedVehicleTypes": allowed_vehicle_types
            }

        return edges

    def get_stations(self, lif_data) -> dict:
        """
        Read all stations except charging stations.
        """
        stations = {}

        for s in lif_data["layouts"][0]["stations"]:
            desc = s.get("stationDescription", "")

            if desc in ["TRANSFER", "PROCESS"]:
                stations[s["stationId"]] = {
                    "interactionNodeIds": s.get("interactionNodeIds", []),
                    "stationDescription": desc
                }

        return stations

    def get_dwelling_nodes(self, lif_data) -> list:
        """
        Collect the IDs of all dwelling / charging nodes.
        """
        dwelling = []

        for s in lif_data["layouts"][0]["stations"]:
            desc = s.get("stationDescription", "")

            if desc == "CHARGING":
                dwelling.extend(s.get("interactionNodeIds", []))

        return dwelling

    # ── Task 2c helper ───────────────────────────────────────────────────────

    def is_edge_allowed(self, edge: dict, vehicle_type_id=None) -> bool:
        """
        Check whether an edge is allowed for a vehicle type.

        If vehicle_type_id is None:
            no filtering is applied.

        If allowedVehicleTypes is empty:
            the edge is treated as usable by all vehicles.

        If allowedVehicleTypes is not empty:
            vehicle_type_id must be inside allowedVehicleTypes.
        """
        if vehicle_type_id is None:
            return True

        allowed_vehicle_types = edge.get("allowedVehicleTypes", [])

        if not allowed_vehicle_types:
            return True

        return vehicle_type_id in allowed_vehicle_types

    # ── Helper methods for A* ────────────────────────────────────────────────

    def get_connected_nodes(self, node_id, vehicle_type_id=None) -> list:
        """
        Return directly connected node IDs.
        """
        connected = []

        for e in self.edges.values():
            if not self.is_edge_allowed(e, vehicle_type_id):
                continue

            if e["startNodeId"] == node_id:
                connected.append(e["endNodeId"])

            elif e["endNodeId"] == node_id:
                connected.append(e["startNodeId"])

        return connected

    def get_connected_edge(self, startNodeId, endNodeId,
                           vehicle_type_id=None) -> str:
        """
        Return the edgeId connecting startNodeId and endNodeId.

        Task 2c update:
        If vehicle_type_id is given, return the edge only if that vehicle type
        is allowed to use it.
        """
        for e in self.edges.values():
            forward_match = (
                e["startNodeId"] == startNodeId
                and e["endNodeId"] == endNodeId
            )

            backward_match = (
                e["startNodeId"] == endNodeId
                and e["endNodeId"] == startNodeId
            )

            if forward_match or backward_match:
                if self.is_edge_allowed(e, vehicle_type_id):
                    return e["edgeId"]

        return None