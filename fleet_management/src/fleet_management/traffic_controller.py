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
            # Dynamic Priority: Calculate estimated time to complete task.
            # Lower score = HIGHER priority (fastest to finish).
            if not hasattr(agent, 'full_nodes') or not hasattr(agent, 'released_index'):
                return (float('inf'), agent.agentId)
            
            remaining_nodes = len(agent.full_nodes) - agent.released_index
            remaining_actions = 0
            for i in range(agent.released_index, len(agent.full_nodes)):
                remaining_actions += len(agent.full_nodes[i].get('actions', []))
            
            # Each action takes roughly as much time as driving 3 nodes
            score = remaining_nodes + (remaining_actions * 3)
            return (score, agent.agentId)

        while self.running:
            # 1. Build registries
            agent_physical = {a: set() for a in self.fleet_manager.agents.agents}
            agent_intent = {a: set() for a in self.fleet_manager.agents.agents}
            
            for a in self.fleet_manager.agents.agents:
                if getattr(a, 'current_node', None):
                    agent_physical[a].add(a.current_node)
                
                if a.agent_state == "EXECUTING" and hasattr(a, 'full_nodes'):
                    current_idx = getattr(a, 'tracked_current_idx', 0)
                    if current_idx >= len(a.full_nodes):
                        current_idx = len(a.full_nodes) - 1
                    
                    found = False
                    current_node_id = getattr(a, 'current_node', None)
                    if current_node_id:
                        for i in range(current_idx, len(a.full_nodes)):
                            if a.full_nodes[i]['nodeId'] == current_node_id:
                                current_idx = i
                                found = True
                                break
                                
                        if not found:
                            for i in range(0, current_idx):
                                if a.full_nodes[i]['nodeId'] == current_node_id:
                                    current_idx = i
                                    break
                    a.tracked_current_idx = current_idx
                    
                    # Physical: current up to released
                    for i in range(current_idx, getattr(a, 'released_index', 0)):
                        agent_physical[a].add(a.full_nodes[i]['nodeId'])
                    
                    # Path tracking for intent checks
                    # Dynamic lookahead: Look ahead only up to the next pending HARD action.
                    # This prevents agents from claiming the entire map and causing premature tie-breaker deadlocks.
                    path_slice = []
                    if getattr(a, 'current_node', None):
                        path_slice.append(a.current_node)
                        agent_intent[a].add(a.current_node) # Explicitly add current node to intent!
                    
                    for i in range(current_idx, len(a.full_nodes)):
                        node_id = a.full_nodes[i]['nodeId']
                        if node_id not in path_slice:
                            path_slice.append(node_id)
                            agent_intent[a].add(node_id)
                            
                        # Stop intent lookahead at first unfinished HARD action
                        node_actions = a.full_nodes[i].get('actions', [])
                        has_pending = False
                        for act in node_actions:
                            if act.get('blockingType') == 'HARD':
                                action_status = next((a_s.get('actionStatus') for a_s in getattr(a, 'actionStates', []) if a_s.get('actionId') == act.get('actionId')), 'WAITING')
                                if action_status != 'FINISHED':
                                    has_pending = True
                                    break
                        if has_pending:
                            break
                    
                    setattr(a, 'tracked_path', path_slice)

            # 2. Try to release MORE nodes for executing agents
            any_blocked = False
            for a in self.fleet_manager.agents.agents:
                if a.agent_state == "EXECUTING" and hasattr(a, 'full_nodes'):
                    current_idx = getattr(a, 'tracked_current_idx', 0)
                    last_current_idx = getattr(a, 'last_current_idx', -1)
                    updated = False
                    
                    if current_idx > last_current_idx:
                        a.last_current_idx = current_idx
                        # Check if the current node has pending actions
                        node_actions = a.full_nodes[current_idx].get('actions', [])
                        has_pending_curr = False
                        for act in node_actions:
                            status = next((a_s.get('actionStatus') for a_s in getattr(a, 'actionStates', []) if a_s.get('actionId') == act.get('actionId')), 'WAITING')
                            if status != 'FINISHED':
                                has_pending_curr = True
                                break
                        # Only trigger update if no pending actions, otherwise it interrupts the simulator
                        if not has_pending_curr:
                            updated = True
                        
                    while getattr(a, 'released_index', 0) < len(a.full_nodes):
                        next_node_id = a.full_nodes[a.released_index]['nodeId']
                        
                        is_blocked = False
                        for other in self.fleet_manager.agents.agents:
                            if other == a: continue
                            
                            priority_a = get_priority(a)
                            priority_other = get_priority(other)
                            
                            # Rule A: Strict physical collision
                            if next_node_id in agent_physical[other]:
                                # Mutual claim on the next node
                                if next_node_id in agent_physical.get(a, set()):
                                    if a.agentId < other.agentId:
                                        print(f"[TrafficController] {a.agentId} blocked from {next_node_id} by Rule A tie-breaker of {other.agentId}")
                                        is_blocked = True
                                        break
                                else:
                                    print(f"[TrafficController] {a.agentId} blocked from {next_node_id} by Rule A (physical) of {other.agentId}")
                                    is_blocked = True
                                    break
                                
                            # Rule B: Dynamic Overlap Deadlock Avoidance
                            intent_other = agent_intent[other]
                            if next_node_id in intent_other:
                                overlap = agent_intent[a].intersection(intent_other)
                                
                                # Find if it's a head-on collision or just crossing
                                is_head_on = False
                                if len(overlap) >= 2:
                                    a_overlap_nodes = [n for n in getattr(a, 'tracked_path', []) if n in overlap]
                                    other_overlap_nodes = [n for n in getattr(other, 'tracked_path', []) if n in overlap]
                                    if len(a_overlap_nodes) >= 2 and len(other_overlap_nodes) >= 2:
                                        u, v = a_overlap_nodes[0], a_overlap_nodes[1]
                                        if other_overlap_nodes.index(u) > other_overlap_nodes.index(v):
                                            is_head_on = True

                                if is_head_on:
                                    # Corridor conflict: must yield before entering the corridor!
                                    my_current = getattr(a, 'current_node', None)
                                    other_current = getattr(other, 'current_node', None)
                                    
                                    # Anti-Deadlock Rule for Bottlenecks:
                                    # Use the node just before next_node_id in my path to see if I am entering from a safe, unshared path.
                                    my_approach_node = a.full_nodes[a.released_index - 1]['nodeId'] if a.released_index > 0 else my_current
                                    
                                    if other_current in agent_intent[a] and my_approach_node not in agent_intent[other]:
                                        print(f"[TrafficController] {a.agentId} blocked from {next_node_id}: yielding to {other.agentId} to clear bottleneck (waiting at {my_approach_node})")
                                        is_blocked = True
                                        break
                                        
                                    am_i_in_overlap = my_current in overlap
                                    is_other_in_overlap = other_current in overlap
                                    
                                    if is_other_in_overlap and not am_i_in_overlap:
                                        print(f"[TrafficController] {a.agentId} blocked from {next_node_id}: {other.agentId} is already in the overlap {overlap}")
                                        is_blocked = True
                                        break
                                    elif am_i_in_overlap and not is_other_in_overlap:
                                        # I am in the overlap, I must proceed to clear it
                                        pass
                                    elif am_i_in_overlap and is_other_in_overlap:
                                        # Both in! Priority decides, but this is a dangerous state.
                                        if priority_a > priority_other: # a is lower priority
                                            print(f"[TrafficController] {a.agentId} blocked from {next_node_id}: yielding to higher priority {other.agentId} (both in overlap)")
                                            is_blocked = True
                                            break
                                        elif priority_a == priority_other and a.agentId < other.agentId:
                                            print(f"[TrafficController] {a.agentId} blocked from {next_node_id}: yielding to tie-breaker {other.agentId} (both in overlap)")
                                            is_blocked = True
                                            break
                                    else:
                                        # Neither in! Priority decides who enters.
                                        if priority_a > priority_other: # a is lower priority
                                            print(f"[TrafficController] {a.agentId} blocked from {next_node_id}: yielding to higher priority {other.agentId} for overlap {overlap}")
                                            is_blocked = True
                                            break
                                        elif priority_a == priority_other:
                                            # If someone ALREADY HAS THE NODE RELEASED, they MUST have priority!
                                            other_released_nodes = [n['nodeId'] for n in getattr(other, 'full_nodes', [])[:getattr(other, 'released_index', 0)]]
                                            if any(node in other_released_nodes for node in overlap):
                                                print(f"[TrafficController] {a.agentId} blocked from {next_node_id}: {other.agentId} already has released permission for overlap {overlap}")
                                                is_blocked = True
                                                break
                                            if a.agentId < other.agentId:
                                                print(f"[TrafficController] {a.agentId} blocked from {next_node_id}: yielding to tie-breaker {other.agentId} for overlap {overlap}")
                                                is_blocked = True
                                                break
                                else:
                                    # Simple crossing or same direction conflict
                                    # We DO NOT need to reserve the entire overlap!
                                    # Rule A (physical collision prevention) will naturally handle simple crossings
                                    # and same-direction following by ensuring a 1-node gap at runtime.
                                    pass

                        if is_blocked:
                            any_blocked = True
                            break
                            
                        # Look ahead limit (only release 3 nodes ahead of current position)
                        current_idx = getattr(a, 'tracked_current_idx', 0)
                        if a.released_index - current_idx >= 3:
                            break

                        # Release it!
                        a.full_nodes[a.released_index]['released'] = True
                        if a.released_index > 0 and a.released_index - 1 < len(getattr(a, 'full_edges', [])):
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
                            nodes=a.full_nodes[current_idx:],
                            edges=getattr(a, 'full_edges', [])[current_idx:],
                            start_sequence_idx=current_idx
                        )
            
            if any_blocked:
                for ag in self.fleet_manager.agents.agents:
                    actions = [a.get('actionStatus') for a in getattr(ag, 'actionStates', [])]
                    print(f"[STATUS-{ag.agentId}] state={ag.agent_state}, curr_node={getattr(ag, 'current_node', None)}, pos={getattr(ag, 'agvPosition', None)}, released_idx={getattr(ag, 'released_index', None)}, actions={actions}")
            
            time.sleep(0.5)
