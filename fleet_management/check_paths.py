import json
import sys
import os

sys.path.insert(0, os.path.join(os.getcwd(), 'src'))
from fleet_management.graph import Graph
from fleet_management.fleet_management import FleetManagement, PathPlanning

with open('data/input_files/lif_file.json') as f:
    lif_data = json.load(f)
graph = Graph(lif_data=lif_data)
pp = PathPlanning(config_data={}, graph=graph)

# Check paths
# mouse001: starts at N7/N5 (configured starting position in agentsInitialization_file.json is N7/N5)
# Actually let's look at agentsInitialization_file.json:
# mouse001: N7 (1.00, 4.71)
# cat001: N12 (7.50, 0.94)

# mouse001 task: N2(pick), N3(process), N4(drop) -> return to dwelling
# cat001 task: N4(pick), N3(process), N2(drop) -> return to dwelling

print("mouse001 path N7 -> N2 -> N3 -> N4 -> dwelling:")
path_nodes_mouse_1, _ = pp.astar_search("N7", "N2", "Lateral_Conveyor")
path_nodes_mouse_2, _ = pp.astar_search("N2", "N3", "Lateral_Conveyor")
path_nodes_mouse_3, _ = pp.astar_search("N3", "N4", "Lateral_Conveyor")
print("  N7 -> N2:", path_nodes_mouse_1)
print("  N2 -> N3:", path_nodes_mouse_2)
print("  N3 -> N4:", path_nodes_mouse_3)

print("cat001 path N12 -> N4 -> N3 -> N2 -> dwelling:")
path_nodes_cat_1, _ = pp.astar_search("N12", "N4", "Longitudinal_Conveyor")
path_nodes_cat_2, _ = pp.astar_search("N4", "N3", "Longitudinal_Conveyor")
path_nodes_cat_3, _ = pp.astar_search("N3", "N2", "Longitudinal_Conveyor")
print("  N12 -> N4:", path_nodes_cat_1)
print("  N4 -> N3:", path_nodes_cat_2)
print("  N3 -> N2:", path_nodes_cat_3)
