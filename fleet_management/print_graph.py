import json
import sys
import os

sys.path.insert(0, os.path.join(os.getcwd(), 'src'))
from fleet_management.graph import Graph

with open('data/input_files/lif_file.json') as f:
    lif_data = json.load(f)
graph = Graph(lif_data=lif_data)

for n_id, node in sorted(graph.nodes.items()):
    print(f"Node {n_id} ({node['pos']}):")
    for other_id in sorted(graph.get_connected_nodes(n_id)):
        edge = graph.get_connected_edge(n_id, other_id)
        edge_data = graph.edges[edge]
        print(f"  -> {other_id} via {edge} (allowed: {edge_data['allowedVehicleTypes']})")
