import json

# 1. config_file.json
with open('fleet_management/data/input_files/config_file.json', 'r') as f:
    config_content = f.read()
config_content = config_content.replace('"simulation_run_time": 6000000', '"simulation_run_time": 80')
with open('fleet_management/data/input_files/config_file.json', 'w') as f:
    f.write(config_content)

# 2. lif_file.json
with open('fleet_management/data/input_files/lif_file.json', 'r') as f:
    lif_content = f.read()
lif_content = lif_content.replace('"rotationAllowed": false', '"rotationAllowed": true')
with open('fleet_management/data/input_files/lif_file.json', 'w') as f:
    f.write(lif_content)

# 3. fleet_management.py
with open('fleet_management/src/fleet_management/fleet_management.py', 'r') as f:
    fm_content = f.read()
target = """                tentative_g_score = (
                    g_score[current_node]
                    + self.get_distance(current_node, neighbour)
                )"""
replacement = """                penalty = 0.0
                if neighbour == 'N11' and goal_node != 'N3':
                    penalty = 5.0
                    
                tentative_g_score = (
                    g_score[current_node]
                    + self.get_distance(current_node, neighbour)
                    + penalty
                )"""
if penalty_check := "penalty = 0.0" not in fm_content:
    fm_content = fm_content.replace(target, replacement)
    with open('fleet_management/src/fleet_management/fleet_management.py', 'w') as f:
        f.write(fm_content)

# 4. order_interface.py
with open('fleet_management/src/vda5050_interface/interfaces/order_interface.py', 'r') as f:
    oi_content = f.read()
if "start_sequence_idx" not in oi_content:
    oi_content = oi_content.replace(
        'def generate_order_message(self, agent: object, orderId: str, order_updateId: int,\n                               nodes: list, edges: list) -> None:',
        'def generate_order_message(self, agent: object, orderId: str, order_updateId: int,\n                               nodes: list, edges: list, start_sequence_idx: int = 0) -> None:'
    )
    oi_content = oi_content.replace(
        '"sequenceId": i * 2,',
        '"sequenceId": (start_sequence_idx + i) * 2,'
    )
    oi_content = oi_content.replace(
        '"sequenceId": i * 2 + 1,',
        '"sequenceId": (start_sequence_idx + i) * 2 + 1,'
    )
    oi_content = oi_content.replace(
        '"headerId": agent.agents.order_header_id,',
        '"headerId": agent.order_header_id,'
    )
    oi_content = oi_content.replace(
        'agent.agents.order_header_id += 1',
        'agent.order_header_id += 1'
    )
    with open('fleet_management/src/vda5050_interface/interfaces/order_interface.py', 'w') as f:
        f.write(oi_content)

# 5. agents.py
with open('fleet_management/src/fleet_management/agents.py', 'r') as f:
    agents_content = f.read()
if "self.order_header_id = 1" not in agents_content:
    target_agents = """        self.order_interface = OrderInterface(
            config_data=self.agents.config_data, logging=logging,
            order_topic=self.order_topic, agentId=self.agentId)"""
    replacement_agents = target_agents + "\n        self.order_header_id = 1"
    agents_content = agents_content.replace(target_agents, replacement_agents)
    with open('fleet_management/src/fleet_management/agents.py', 'w') as f:
        f.write(agents_content)

print("Restored all config and interface files.")
