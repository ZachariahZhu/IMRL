class Graph:
    """
    Class representing the layout as a graph.
    Nodes, edges, stations, and dwelling nodes are read from the LIF file.

    Task 4: Model the layout as a graph.
    Implement the four methods below so that a pathfinding algorithm can later find
    the shortest path between any two nodes in the layout.
    """

    def __init__(self, lif_data):
        self.nodes = self.get_nodes(lif_data)
        self.edges = self.get_edges(lif_data)
        self.stations = self.get_stations(lif_data)
        self.dwelling_nodes = self.get_dwelling_nodes(lif_data)

    # ── Task 4 ────────────────────────────────────────────────────────────────

    def get_nodes(self, lif_data) -> dict:
        """
        Read all nodes from the LIF file.

        Return a dict mapping nodeId (str) -> node info dict.
        The node info dict must contain at least:
            'nodeId' : str    - the node ID
            'pos'    : (x, y) - the node's position in meters

        Hint: nodes are listed under lif_data['layouts'][0]['nodes'].
              Each node has a 'nodeId' and 'nodePosition' with 'x' and 'y'.
        """
        # TODO Task 4: Implement this method.
        pass

    def get_edges(self, lif_data) -> dict:
        """
        Read all edges from the LIF file.

        Return a dict mapping edgeId (str) -> edge info dict.
        The edge info dict must contain at least:
            'edgeId'       : str    - the edge ID
            'startNodeId'  : str    - the start node ID (from LIF definition)
            'endNodeId'    : str    - the end node ID (from LIF definition)
            'startNodePos' : (x, y) - position of the start node
            'endNodePos'   : (x, y) - position of the end node

        Note: All edges in this layout are bidirectional — the robot can
              travel in either direction along every edge.

        Hint: edges are listed under lif_data['layouts'][0]['edges'].
              Use self.nodes[nodeId]['pos'] to look up node positions.
        """
        # TODO Task 4: Implement this method.
        pass

    def get_stations(self, lif_data) -> dict:
        """
        Read all stations (pick/drop/process locations) from the LIF file.

        Return a dict mapping stationId (str) -> station info dict.
        The station info dict must contain at least:
            'interactionNodeIds'  : list[str] - node IDs where the robot interacts
            'stationDescription'  : str       - 'TRANSFER' or 'PROCESS'

        The stationDescription is used by FleetManagement to decide which actions
        to assign:
            'TRANSFER' → 'init_fine_positioning' on the preceding node + 'pick'/'drop'
            'PROCESS'  → 'process' action on the station node

        Hint: stations are listed under lif_data['layouts'][0]['stations'].
              Include only stations with stationDescription 'TRANSFER' or 'PROCESS'.
              Stations with stationDescription 'CHARGING' are dwelling nodes (see below).
        """
        # TODO Task 4: Implement this method.
        pass

    def get_dwelling_nodes(self, lif_data) -> list:
        """
        Collect the IDs of all dwelling (charging/waiting) nodes.

        Return a list of nodeId strings.
        Dwelling nodes are identified by stationDescription == 'CHARGING' in the LIF.
        The robot starts at a dwelling node and returns to one after completing all tasks.

        Hint: same station list as get_stations(), just filter for 'CHARGING'.
        """
        # TODO Task 4: Implement this method.
        pass

    # ── Helper methods (needed for Task 6 – A*) ──────────────────────────────

    def get_connected_nodes(self, node_id) -> list:
        """
        Return a list of node IDs directly connected to node_id via any edge.
        Edges are bidirectional: check both startNodeId and endNodeId.

        Required by the A* algorithm to explore neighbors.
        """
        # TODO Task 4: Implement this method.
        pass

    def get_connected_edge(self, startNodeId, endNodeId) -> str:
        """
        Return the edgeId connecting startNodeId and endNodeId (in either direction).
        Return None if no such edge exists.

        Required by the A* algorithm to build path_edges from a found path (list of node IDs).
        """
        # TODO Task 4: Implement this method.
        pass
