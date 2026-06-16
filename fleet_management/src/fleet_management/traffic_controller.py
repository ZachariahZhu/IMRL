import time
import threading

class TrafficController:
    """
    Optimization Scheme 2: Dynamic Zone Control (Order Update).
    Dynamically releases nodes to vehicles to prevent deadlocks and collisions.
    """
    def __init__(self, fleet_manager):
        self.fleet_manager = fleet_manager
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        def get_priority(agent):
            # Dynamic Priority: (Remaining Nodes, Agent ID)
            # Lower tuple = HIGHER priority. 
            # "更容易完成任务的机器人有优先权" -> fewer remaining nodes gets priority!
            if not hasattr(agent, 'full_nodes'):
                return (float('inf'), agent.agentId)
            remaining = len(agent.full_nodes) - getattr(agent, 'released_index', 0)
            return (remaining, agent.agentId)

        while self.running:
            # 1. Build registries
            agent_physical = {} # Nodes currently occupied or explicitly released
            agent_intent = {}   # All future nodes in the path
            
            for a in self.fleet_manager.agents.agents:
                agent_physical[a] = set()
                agent_intent[a] = set()
                
                if getattr(a, 'current_node', None):
                    agent_physical[a].add(a.current_node)
                
                if a.agent_state == "EXECUTING" and hasattr(a, 'full_nodes'):
                    current_idx = getattr(a, 'tracked_current_idx', 0)
                    for i in range(current_idx, len(a.full_nodes)):
                        if a.full_nodes[i]['nodeId'] == getattr(a, 'current_node', None):
                            current_idx = i
                            break
                    a.tracked_current_idx = current_idx
                    
                    # Physical: current up to released
                    for i in range(current_idx, getattr(a, 'released_index', 0)):
                        agent_physical[a].add(a.full_nodes[i]['nodeId'])
                    
                    # Path tracking for intent checks
                    # Limit to next 3 nodes to avoid looking past stations and causing false positive traps
                    path_slice = []
                    if getattr(a, 'current_node', None):
                        path_slice.append(a.current_node)
                    for i in range(current_idx, min(len(a.full_nodes), current_idx + 3)):
                        if a.full_nodes[i]['nodeId'] not in path_slice: # Keep first occurrence
                            path_slice.append(a.full_nodes[i]['nodeId'])
                            agent_intent[a].add(a.full_nodes[i]['nodeId'])
                    setattr(a, 'tracked_path', path_slice)

            # 2. Try to release MORE nodes for executing agents
            for a in self.fleet_manager.agents.agents:
                if a.agent_state == "EXECUTING" and hasattr(a, 'full_nodes'):
                    updated = False
                    
                    while getattr(a, 'released_index', 0) < len(a.full_nodes):
                        next_node_id = a.full_nodes[a.released_index]['nodeId']
                        
                        is_blocked = False
                        for other in self.fleet_manager.agents.agents:
                            if other == a: continue
                            
                            # Determine dynamic priorities
                            priority_a = get_priority(a)
                            priority_other = get_priority(other)
                            
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
                        if is_blocked:
                            break
                            
                        # Look ahead limit (only release 3 nodes ahead of current position)
                        current_idx = getattr(a, 'tracked_current_idx', 0)
                        if a.released_index - current_idx >= 3:
                            break

                        # Release it!
                        a.full_nodes[a.released_index]['released'] = True
                        if a.released_index - 1 < len(getattr(a, 'full_edges', [])):
                            a.full_edges[a.released_index - 1]['released'] = True
                        
                        agent_physical[a].add(next_node_id)
                        a.released_index += 1
                        updated = True
                        
                    if updated:
                        a.order_update_id += 1
                        self.fleet_manager.agents.logging.info(f"[TrafficController] Sending OrderUpdate {a.order_update_id} to {a.agentId}, releasing up to index {a.released_index}")
                        a.order_interface.generate_order_message(
                            agent=a,
                            orderId=getattr(a, 'current_order_id', str(self.fleet_manager.agents.order_header_id)),
                            order_updateId=a.order_update_id,
                            nodes=a.full_nodes,
                            edges=getattr(a, 'full_edges', [])
                        )
            time.sleep(0.5)
