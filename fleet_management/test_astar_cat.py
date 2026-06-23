import json
import sys
sys.path.append('src')
from fleet_management.graph import Graph
from fleet_management.fleet_management import FleetManagement

with open('data/input_files/lif_file.json', 'r') as f:
    lif_data = json.load(f)
    
graph = Graph(lif_data)
fm = FleetManagement(None, graph, None, None, None)

nodes, edges = fm.path_planning.astar_search('N12', 'N4', 'Longitudinal_Conveyor')
print(f"Path N12->N4 for Longitudinal_Conveyor: nodes={nodes}, edges={edges}")

nodes, edges = fm.path_planning.astar_search('N12', 'N2', 'Longitudinal_Conveyor')
print(f"Path N12->N2 for Longitudinal_Conveyor: nodes={nodes}, edges={edges}")

