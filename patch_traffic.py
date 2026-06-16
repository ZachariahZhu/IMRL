import re

with open('fleet_management/src/fleet_management/traffic_controller.py', 'r') as f:
    content = f.read()

# Replace the inner block
new_logic = """
                            # Rule A: Strict physical collision
                            if next_node_id in agent_physical[other]:
                                is_blocked = True
                                break
                                
                            # Rule B: Simple Deadlock Avoidance
                            if next_node_id in agent_intent[other]:
                                if a.agentId < other.agentId:
                                    # I am lower priority. I yield.
                                    if agent_physical[a].intersection(agent_intent[other]):
                                        pass # Escape clause
                                    else:
                                        is_blocked = True
                                        break
                                else:
                                    # I am higher priority.
                                    if agent_physical[other].intersection(agent_intent[a]) and next_node_id in agent_intent[other]:
                                        is_blocked = True
                                        break
"""

# Find the start and end of the rules block
start_str = "# Rule A: Strict physical collision"
end_str = "if is_blocked:"

start_idx = content.find(start_str)
end_idx = content.find(end_str)

if start_idx != -1 and end_idx != -1:
    patched = content[:start_idx] + new_logic.strip() + "\n                        " + content[end_idx:]
    with open('fleet_management/src/fleet_management/traffic_controller.py', 'w') as f:
        f.write(patched)
        print("Patched successfully!")
else:
    print("Could not find block to patch.")
