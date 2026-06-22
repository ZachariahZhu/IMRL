import math
import uuid
import time
import threading
import heapq

FIXED_COLUMS=16
edgeCosts = {
    "E1": 3,
    "E2":4,
    "E3":5,
    "E4": 3,
    "E5":4.6,
    "E6":5.9,
    "E7": 3.4,
    "E8":7,
    "E9":5,
    "E10": 3.4,
    "E11":4.8,
    "E12":5.4,
    "E13": 3.8,
    "E14":4.4,
    "E15":5,
    "E16": 3,
    "E17":4,
    "E18":5,
    "E19": 3,
    "E20":4,
    "E21":5,
}
actionCosts = {
    "init_fine_positioning": 5,
    "pick": 4,
    "drop": 4,
    "process": 4
}
nodeCosts = {
    "N1": 0.5,
    "N2": 0.5,
    "N3": 0.5,
    "N4": 0.5,
    "N5": 0.5,
    "N6": 0.5,
    "N7": 0.5,
    "N8": 0.5,
    "N9": 0.5,
    "N10": 0.5,
    "N11": 0.5,
    "N12": 0.5,
    "N13": 0.5,
    "N14": 0.5,
    "N15": 0.5,
    "N16": 0.5,
    "N17": 0.5,
    "N18": 0.5,
    "N19": 0.5,
    "N20": 0.5,
    "N21": 0.5,
    "N22": 0.5,
    "N23": 0.5,
    "N24": 0.5,
    "N25": 0.5
}




class FleetManagement:
    """
    Manages the fleet: computes paths and sends VDA 5050 orders to agents.

    Task 7: Integrate A* into the full order pipeline (multi-stop tasks).
        Implement the helper methods build_path_for_task(), build_order_nodes(),
        and build_order_edges(), then call them from fleet_manager().

    Task 10: Automate all transportation tasks.
        Run fleet_manager() in a loop inside a daemon thread so tasks are
        picked up and executed one after another until all are done.
    """

    def __init__(self, config_data, graph, agents, task_management,
                 simulation_start_time) -> None:
        self.simulation_start_time = simulation_start_time
        self.config_data = config_data
        self.graph = graph
        self.agents = agents
        self.task_management = task_management
        self.path_planning = PathPlanning(config_data=self.config_data,
                                         graph=self.graph)
        threading.Thread(target=self.fleet_manager, daemon=True).start()
        

    def getOccupancyMatrix(self,pNodes,pEdges,stepSize):
        edgeList = []
        nodeList = []
        edgeCostsList = []
        nodeName = []
        nodesAndActions = []
        with open("output.txt", "w") as f: 
           print("New run", file=f)
        for i in range(len(pNodes)):
            nodeName.append(pNodes[i].get("nodeId"))
            #actionList.append(nodes[i].get("actions"))
            if len(pNodes[i].get("actions"))>0:
                   nodesAndActions.append((pNodes[i].get("nodeId"),pNodes[i].get("actions"),i))
            else:
                pass
            
        for i in range(len(pEdges)):
            edgeList.append(pEdges[i].get("edgeId"))
        for e in edgeList:
            edgeCostsList.append(edgeCosts.get(e))

            #for n in nodeName:
            #    pass
        timeWindow = []
        timer = 0
        counter=0
        for n in nodeName:
            if counter == 0:
               timer = 0.5
            else:
                timer = timer+ edgeCostsList[counter-1]
            tempL=[]
            for nAA in nodesAndActions:
                #tempL.append(nAA[2])
               if counter == nAA[2]:    
                    timer = timer+ actionCosts[nAA[1][0].get("actionType")]
                    pass    


            counter=counter +1
            timeWindow.append((n,timer))   


        matrix = []
            # Build rows where, at each time step, the current and next two
            # upcoming nodes (3-node sliding window) are marked as 1.
            # Ensure timeWindow is sorted by activation time.
        timeWindow_sorted = sorted(timeWindow, key=lambda x: x[1])
        step = 0.0
        if timeWindow_sorted:
            last_time = timeWindow_sorted[-1][1]
        else:
            last_time = 0.0

        while step < last_time:
            newRow = [0] * FIXED_COLUMS
            # upcoming nodes whose activation time is at or after current step
            upcoming = [tW for tW in timeWindow_sorted if tW[1] >= step]
            for tW in upcoming[:3]:
                try:
                    idx = int(tW[0][1:])
                except Exception:
                    continue
                if 0 <= idx < FIXED_COLUMS:
                    newRow[idx] = 1
            matrix.append(newRow)
            step += stepSize

        return matrix


    def fuzzyfy(self,inputMatrix,grace,stepSize,initialWidth,witdhIncreasePerSecond):
        """Fuzzyfy matrix edges with a trapezoidal transition around 0/1 boundaries.

        Args:
            inputMatrix: list of rows with binary values (0 or 1).
            grace: seconds before any fuzzyfication starts.
            stepSize: seconds per row increment.
            initialWidth: starting width of the fuzzy transition zone in seconds.
            witdhIncreasePerSecond: linear growth rate of the fuzzy zone width.

        Returns:
            A matrix of floats in [0, 1] with fuzzy transitions applied.
        """
        if stepSize <= 0:
            raise ValueError("stepSize must be positive")

        rows = len(inputMatrix)
        if rows == 0:
            return []

        cols = len(inputMatrix[0])
        # Convert current matrix to floats and preserve original values
        fuzzy_matrix = [[float(value) for value in row] for row in inputMatrix]

        def time_for_index(idx):
            return idx * stepSize

        def width_for_time(edge_time):
            if edge_time <= grace:
                return initialWidth
            return initialWidth + (edge_time - grace) * witdhIncreasePerSecond

        for col in range(cols):
            col_values = [inputMatrix[row][col] for row in range(rows)]
            row_idx = 0
            while row_idx < rows:
                if col_values[row_idx] != 1:
                    row_idx += 1
                    continue

                start_idx = row_idx
                while row_idx < rows and col_values[row_idx] == 1:
                    row_idx += 1
                end_idx = row_idx

                start_time = time_for_index(start_idx)
                end_time = time_for_index(end_idx)

                left_width = width_for_time(start_time)
                right_width = width_for_time(end_time)

                left_start = start_time - left_width
                right_end = end_time + right_width

                for r in range(rows):
                    t = time_for_index(r)
                    if t < grace:
                        continue

                    if left_start <= t < start_time and left_width > 0:
                        ratio = (t - left_start) / left_width
                        fuzzy_value = max(0.0, min(1.0, ratio))
                    elif start_time <= t < end_time:
                        fuzzy_value = 1.0
                    elif end_time <= t <= right_end and right_width > 0:
                        ratio = 1.0 - ((t - end_time) / right_width)
                        fuzzy_value = max(0.0, min(1.0, ratio))
                    else:
                        continue

                    fuzzy_matrix[r][col] = max(fuzzy_matrix[r][col], round(fuzzy_value,2))

        return fuzzy_matrix
    
    def multiplyMatrixes(self,matrix1,matrix2):
        """Multiply two matrices over their overlapping region.

        If one matrix is shorter in rows or columns, the result includes only
        the overlapping portion.
        """
        if matrix1 is None or matrix2 is None:
            return []

        rows1 = len(matrix1)
        rows2 = len(matrix2)
        if rows1 == 0 or rows2 == 0:
            return []

        cols1 = len(matrix1[0]) if rows1 else 0
        cols2 = len(matrix2[0]) if rows2 else 0
        if cols1 == 0 or cols2 == 0:
            return []

        rows = min(rows1, rows2)
        cols = min(cols1, cols2)

        result = []
        for r in range(rows):
            row1 = matrix1[r]
            row2 = matrix2[r]
            result.append([row1[c] * row2[c] for c in range(cols)])

        return result
    
    def vectorize(self, matrix):
        """Sum all elements of each row into a vector.
        
        Args:
            matrix: list of lists representing the matrix
            
        Returns:
            vector (list) where each element is the sum of the corresponding row
        """
        if matrix is None or len(matrix) == 0:
            return []
        
        return [float(sum(row)) for row in matrix]
        
    def limsumVec(self,vec):
        rV = []
        for v in vec:
            if float(v) < 1.:
                rV.append(float(v))
            else:
                rV.append(1.)
        


        return rV
    def visualizeVector(self, vec, timestep):
        """Visualize a vector as colors from white (0) to red (1).

        Uses `limsumVec` to cap values to 1. The `timestep` argument is used
        to attenuate the intensity over time (simple linear decay).

        Args:
            vec: iterable of numeric values (0..inf)
            timestep: numeric time (seconds) used to attenuate intensity

        Returns:
            List of hex color strings (e.g. '#ff7f7f') corresponding to
            each element in `vec` where 0 -> white, 1 -> red.
        """
        if vec is None:
            return []

        # Cap values to maximum 1 using existing helper
        capped = self.limsumVec(vec)

        # Temporal attenuation: larger timesteps reduce intensity slightly.
        # Choose a small decay rate so timestep affects but does not remove
        # signal immediately. Clamp between 0 and 1.
        decay_rate = 0.02
        time_factor = max(0.0, 1.0 - float(timestep) * decay_rate) if timestep is not None else 1.0

        colors = []
        for v in capped:
            intensity = max(0.0, min(1.0, v * time_factor))

            # Interpolate between white (1,1,1) and red (1,0,0):
            # R stays 1, G and B become (1 - intensity)
            r = 1.0
            g = 1.0 - intensity
            b = 1.0 - intensity

            # Convert to 0-255 ints and format hex
            ri = int(round(r * 255))
            gi = int(round(g * 255))
            bi = int(round(b * 255))
            colors.append('#{:02x}{:02x}{:02x}'.format(ri, gi, bi))

        return colors
    def visualizeVectorGUI(self, vec, timestep, title="Vector Visualization",
                           auto_update=False, get_vector=None, update_interval=500,
                           snapshot_dir="data/snapshots"):
        """Display the vector as colored bars in a Tkinter GUI, with optional
        auto-update and snapshot saving.

        Args:
            vec: initial vector to display
            timestep: initial timestep value
            title: window title
            auto_update: if True, periodically call `get_vector` to obtain
                (vec, timestep) updates and refresh the display when changed
            get_vector: callable returning either `vec` or `(vec, timestep)`
            update_interval: milliseconds between polls when auto_update True
            snapshot_dir: directory where snapshot images are saved
        """
        try:
            import tkinter as tk
        except Exception:
            colors = self.visualizeVector(vec, timestep)
            print("Colors:", colors)
            return

        import os
        from datetime import datetime

        # prepare snapshot directory
        try:
            os.makedirs(snapshot_dir, exist_ok=True)
        except Exception:
            pass

        colors = self.visualizeVector(vec, timestep)
        n = len(colors)
        if n == 0:
            print("visualizeVectorGUI: empty vector")
            return

        # Dimensions
        bar_width = max(10, int(800 / n))
        width = bar_width * n
        height = 160

        root = tk.Tk()
        root.title(title)

        canvas = tk.Canvas(root, width=width, height=height, bg='white')
        canvas.pack()

        label_var = tk.StringVar()
        label_var.set(f"timestep: {timestep}")
        lbl = tk.Label(root, textvariable=label_var)
        lbl.pack()

        def draw(colors_list):
            canvas.delete('all')
            for i, col in enumerate(colors_list):
                x0 = i * bar_width
                x1 = x0 + bar_width
                canvas.create_rectangle(x0, 0, x1, height - 40, fill=col, outline='black')
                canvas.create_text(x0 + bar_width / 2, height - 20, text=f"{i}", font=(None, 8))

        def save_snapshot(colors_list, ts):
            # Try Pillow first for PNG, fall back to postscript
            filename_base = datetime.utcnow().strftime("%Y%m%dT%H%M%S.%fZ")
            try:
                from PIL import Image, ImageDraw
                img = Image.new('RGB', (width, height), color='white')
                draw_img = ImageDraw.Draw(img)
                for i, col in enumerate(colors_list):
                    x0 = i * bar_width
                    x1 = x0 + bar_width
                    draw_img.rectangle([x0, 0, x1, height - 40], fill=col, outline='black')
                # Draw timestep text at bottom
                draw_img.text((4, height - 38), f"timestep: {ts}", fill='black')
                out_path = os.path.join(snapshot_dir, f"snapshot_{filename_base}.png")
                img.save(out_path)
            except Exception:
                try:
                    ps_path = os.path.join(snapshot_dir, f"snapshot_{filename_base}.ps")
                    canvas.postscript(file=ps_path)
                except Exception:
                    # give up silently
                    pass

        # initial draw and snapshot
        draw(colors)
        save_snapshot(colors, timestep)

        current_vec = list(vec)
        current_timestep = timestep

        stop_flag = {'stop': False}

        def on_close():
            stop_flag['stop'] = True
            root.destroy()

        root.protocol('WM_DELETE_WINDOW', on_close)

        def poll():
            if stop_flag['stop']:
                return
            updated = False
            nonlocal current_vec, current_timestep
            if auto_update:
                if not callable(get_vector):
                    # nothing to poll
                    root.after(update_interval, poll)
                    return

                try:
                    res = get_vector()
                    if isinstance(res, tuple) or isinstance(res, list):
                        v_new = list(res[0])
                        t_new = res[1] if len(res) > 1 else current_timestep
                    else:
                        v_new = list(res)
                        t_new = current_timestep
                except Exception:
                    root.after(update_interval, poll)
                    return

                if v_new != current_vec or t_new != current_timestep:
                    current_vec = v_new
                    current_timestep = t_new
                    cols = self.visualizeVector(current_vec, current_timestep)
                    draw(cols)
                    label_var.set(f"timestep: {current_timestep}")
                    save_snapshot(cols, current_timestep)

            root.after(update_interval, poll)

        # Start polling if requested
        if auto_update:
            root.after(update_interval, poll)

        # Close button
        btn = tk.Button(root, text="Close", command=on_close)
        btn.pack(pady=4)

        root.mainloop()

    def fleet_manager(self) -> None:
        last_matrix=None
        """
        Send a movement order to the first agent.

        Every transportation task follows this movement structure:
            dwelling (start) -> pick station -> process station(s) -> drop station -> dwelling

        Task 3 — manually fill the 'nodes' and 'edges' lists for the first
        transportation task (T1) and pass them to generate_order_message().

        Fill each entry following these rules:
            Node dict:  {"nodeId": str, "x": float, "y": float,
                         "theta": float | None, "actions": list}
            Edge dict:  {"edgeId": str, "startNodeId": str, "endNodeId": str,
                         "actions": list}

        Action rules (see orderMessage_Example.json for a reference):
          - Node immediately BEFORE a TRANSFER station (pick/drop):
                {"actionType": "init_fine_positioning",
                 "actionId": "<unique-uuid-string>", "blockingType": "HARD"}
          - TRANSFER station node (actionType pick or drop):
                {"actionType": "pick",  "actionId": "...", "blockingType": "HARD"}
             or {"actionType": "drop",  "actionId": "...", "blockingType": "HARD"}
          - PROCESS station node:
                {"actionType": "process", "actionId": "...", "blockingType": "HARD",
                 "processingTime": <float from task>}

        Node positions and theta values can be read from lif_file.json or
        self.graph.nodes[node_id] (if Task 4 is implemented).
        Use str(uuid.uuid4()) to generate unique actionId strings.

        """
                #              The full journey must cover:
        #              N5->N2(pick)->N3(process)->N4(drop)->N14
        # nodes = [{"nodeId":"N5", "x": 1.05, "y": 0.9, "theta": None, "actions": []},
        #          {"nodeId": "N1", "x": 2.46, "y": 0.79, "theta": -1.57, "actions": []},
        #          #N7(fine positioning) 
        #          {"nodeId": "N7", "x": 2.46, "y": 2.25, "theta": None, "actions": [{"actionType": "init_fine_positioning", "actionId": str(uuid.uuid4()), "blockingType": "HARD"}]},
        #          {"nodeId": "N2", "x": 2.46, "y": 3.7, "theta": 1.57, "actions": [{"actionType": "pick", "actionId": str(uuid.uuid4()), "blockingType": "HARD"}]},
        #          {"nodeId": "N7", "x": 2.46, "y": 2.25, "theta": None, "actions": []},
        #          {"nodeId": "N8", "x": 4.0, "y": 2.25, "theta": None, "actions": []},
        #          #N3(process)
        #          {"nodeId": "N3", "x": 5.45, "y": 2.25, "theta": None, "actions": [{"actionType": "process", "actionId": str(uuid.uuid4()), "blockingType": "HARD", "processingTime": 1.0}]},
        #          {"nodeId": "N13", "x": 5.55, "y": 3.5, "theta": None, "actions": []},
        #             #N4(drop，N12 fine positioning)
        #          {"nodeId": "N12", "x": 4.0, "y": 3.5, "theta": None, "actions": [{"actionType": "init_fine_positioning", "actionId": str(uuid.uuid4()), "blockingType": "HARD"}]},
        #          {"nodeId": "N4", "x": 5.55, "y": 4.75, "theta": 0.0, "actions": [{"actionType": "drop", "actionId": str(uuid.uuid4()), "blockingType": "HARD"}]},
        #          {"nodeId": "N12", "x": 4.0, "y": 3.5, "theta": None, "actions": []},
        #          {"nodeId": "N13", "x": 5.55, "y": 3.5, "theta": None, "actions": []},
        #          {"nodeId": "N14", "x": 6.8, "y": 3.5, "theta": None, "actions": []}  # RETURN to dwelling
        # ]   # List of node dicts with nodeId, x, y, theta, actions
        # edges = [
        #     {"edgeId": "E21", "startNodeId": "N5", "endNodeId": "N1", "actions": []},
        #     {"edgeId": "E1", "startNodeId": "N1", "endNodeId": "N7", "actions": []},
        #     {"edgeId": "E2", "startNodeId": "N7", "endNodeId": "N2", "actions": []},
        #     {"edgeId": "E2", "startNodeId": "N2", "endNodeId": "N7", "actions": []},
        #     {"edgeId": "E3", "startNodeId": "N7", "endNodeId": "N8", "actions": []},
        #     {"edgeId": "E5", "startNodeId": "N8", "endNodeId": "N3", "actions": []},
        #     {"edgeId": "E17", "startNodeId": "N3", "endNodeId": "N13", "actions": []},
        #     {"edgeId": "E12", "startNodeId": "N13", "endNodeId": "N12", "actions": []},
        #     {"edgeId": "E19", "startNodeId": "N12", "endNodeId": "N4", "actions": []},
        #     {"edgeId": "E19", "startNodeId": "N4", "endNodeId": "N12", "actions": []},
        #     {"edgeId": "E12", "startNodeId": "N12", "endNodeId": "N13", "actions": []},
        #     {"edgeId": "E16", "startNodeId": "N13", "endNodeId": "N14", "actions": []}
               # List of edge dicts with edgeId, startNodeId, endNodeId, actions
        """
        Task 7 — replace the manual lists with A*:
            1. Pick the first unassigned task from self.task_management.task_list.
            2. Call self.build_path_for_task(task, agent.current_node).
            3. Call self.build_order_nodes(path_nodes, task) and
               self.build_order_edges(path_nodes, path_edges).
            4. Pass the results to generate_order_message() below.
            5. Mark task['task_assigned'] = True and agent.agent_state = 'EXECUTING'.
        """
        # task= next(t for t in self.task_management.task_list if not t['task_assigned'])#1.找到第一个未分配的任务
        # agent= self.agents.agents[0]#2.找到一个空闲车
        # path_nodes,path_edges = self.build_path_for_task(task, agent.current_node)
        # nodes= self.build_order_nodes(path_nodes, task)
        # edges= self.build_order_edges(path_nodes, path_edges)#3.自动路径
        # task['task_assigned'] = True
        # agent.agent_state = 'EXECUTING'#4.锁定车和任务
        # agent.order_interface.generate_order_message(
        #     agent=agent,
        #     orderId=str(self.agents.order_header_id),
        #     order_updateId=0,
        #     nodes=nodes,
        #     edges=edges#发送订单
        # )


        import time
        while any(not t['task_assigned'] for t in self.task_management.task_list):
            
            # Find all idle agents
            idle_agents = [a for a in self.agents.agents if a.agent_state == "IDLE"]
            
            if not idle_agents:
                time.sleep(0.5)
                continue
                
            # Pick the first idle agent
            agent = idle_agents[0]
                
            try:
                task = next(t for t in self.task_management.task_list if not t['task_assigned'])
            except StopIteration:
                break
                
            path_nodes, path_edges = self.build_path_for_task(task, agent.current_node)
            nodes = self.build_order_nodes(path_nodes, task)
            edges = self.build_order_edges(path_nodes, path_edges)

            """ edgeList = []
            nodeList = []
            edgeCostsList = []
            nodeName = []
            nodesAndActions = []
            with open("output.txt", "w") as f: 
                print("New run", file=f)
            for i in range(len(nodes)):
                nodeName.append(nodes[i].get("nodeId"))
                #actionList.append(nodes[i].get("actions"))
                if len(nodes[i].get("actions"))>0:
                    nodesAndActions.append((nodes[i].get("nodeId"),nodes[i].get("actions"),i))
                else:
                    pass
            
            for i in range(len(edges)):
                edgeList.append(edges[i].get("edgeId"))

            for e in edgeList:
                edgeCostsList.append(edgeCosts.get(e))

            #for n in nodeName:
            #    pass
            timeWindow = []
            timer = 0
            counter=0
            for n in nodeName:
                if counter == 0:
                    timer = 0.5
                else:
                    timer = timer+ edgeCostsList[counter-1]
                tempL=[]
                for nAA in nodesAndActions:
                    #tempL.append(nAA[2])
                    if counter == nAA[2]:
                        timer = timer+ actionCosts[nAA[1][0].get("actionType")]
                        pass    


                counter=counter +1
                timeWindow.append((n,timer))   


            matrix = []
            # Build rows where, at each time step, the current and next two
            # upcoming nodes (3-node sliding window) are marked as 1.
            # Ensure timeWindow is sorted by activation time.
            timeWindow_sorted = sorted(timeWindow, key=lambda x: x[1])
            step = 0.0
            if timeWindow_sorted:
                last_time = timeWindow_sorted[-1][1]
            else:
                last_time = 0.0

            while step < last_time:
                newRow = [0] * FIXED_COLUMS
                # upcoming nodes whose activation time is at or after current step
                upcoming = [tW for tW in timeWindow_sorted if tW[1] >= step]
                for tW in upcoming[:3]:
                    try:
                        idx = int(tW[0][1:])
                    except Exception:
                        continue
                    if 0 <= idx < FIXED_COLUMS:
                        newRow[idx] = 1
                matrix.append(newRow)
                step += 0.5


            with open("output.txt", "a") as f: 
                #print("test")
                #print(nodeName,file=f)
                #print(actionList,file=f)
                print(nodesAndActions,file=f)
                for i in range(len(nodesAndActions)):
                    print(nodesAndActions[i][0],nodesAndActions[i][1][0].get("actionType"),actionCosts[nodesAndActions[i][1][0].get("actionType")],file=f)
                print(edgeList,file=f)
                print(edgeCostsList,file=f)
                print(nodeName,file=f)
                print(timeWindow,file=f)
                print(matrix,file=f)
                for row in matrix:
                    for item in row:
                        print(item,end=" ",file=f)
                    print(file=f) """
            with open("output2.txt","w") as fff:
                print (self.getOccupancyMatrix(nodes,edges,0.2),file=fff)
                for row in self.getOccupancyMatrix(nodes,edges,0.2):
                    for item in row:
                        print(item,end=" ",file=fff)
                    print(file=fff)
                for row in self.fuzzyfy(self.getOccupancyMatrix(nodes,edges,0.2),10,0.2,1,0.05):
                    for item in row:
                        print(item,end=" ",file=fff)
                    print(file=fff)
            combine=None
            if last_matrix != None:
                combine=self.multiplyMatrixes(self.fuzzyfy(self.getOccupancyMatrix(nodes,edges,0.2),10,0.2,1,0.05),last_matrix)
            
            last_matrix=self.fuzzyfy(self.getOccupancyMatrix(nodes,edges,0.2),10,0.2,1,0.05)
            with open("output3.txt","w") as ffff:
                if combine != None:
                    print(combine,file=ffff)
                    for row in combine:
                            for item in row:
                                print(item,end=" ",file=ffff)
                            print(file=ffff)
                print(self.vectorize(combine),file=ffff)
            if combine!= None:
                self.visualizeVectorGUI(self.visualizeVector(self.vectorize(combine),0.2), 0.2)

            
            
            task['task_assigned'] = True
            agent.agent_state = 'EXECUTING'
            agent.current_task = task
            
            agent.order_interface.generate_order_message(
                agent=agent,
                orderId=str(self.agents.order_header_id),
                order_updateId=0,
                nodes=nodes,
                edges=edges
            )
            time.sleep(0.5)

    def build_path_for_task(self, task: dict, start_node: str) -> tuple:
        """
        Task 7: Chain multiple A* searches to cover all stations in a task.

        Every task follows: dwelling -> pick -> process(es) -> drop -> dwelling

        A task's stations list covers only the waypoints (not the dwelling legs):
            [{"nodeId": "N4", "actionType": "pick"},
             {"nodeId": "N3", "actionType": "process", "processingTime": 1.0},
             {"nodeId": "N2", "actionType": "drop"}]

        Build the full path by planning one A* leg per station plus a return leg:
            Leg 0: start_node  -> stations[0]['nodeId']   (travel to pick)
            Leg 1: stations[0] -> stations[1]['nodeId']   (pick to process)
            Leg 2: stations[1] -> stations[2]['nodeId']   (process to drop)
            Return: last station -> nearest dwelling node

        After each astar_search() call, extend the combined lists by:
            - path_nodes: skip the first node of each new leg (it duplicates
              the last node of the previous leg).
            - path_edges: concatenate as-is.

        For the return leg, choose the nearest dwelling node from
        self.graph.dwelling_nodes using self.graph.nodes[n]['pos'].

        Returns (path_nodes, path_edges).
        """
        combined_nodes= []
        combined_edges= []
        current= start_node
        #1.规划每一站的路径
        for station in task['stations']:
            target_node= station['nodeId']
            nodes,edges= self.path_planning.astar_search(current, target_node)

            if not combined_nodes:
                combined_nodes.extend(nodes)
            else:
                combined_nodes.extend(nodes[1:])#跳过第一个节点，避免重复
            combined_edges.extend(edges)
            current= target_node

        #2.找到最近的休息点
        nearest_dwelling= None
        min_dist = float('inf')
        pos_current = self.graph.nodes[current]['pos']
        
        for d_node in self.graph.dwelling_nodes:
            pos_d=self.graph.nodes[d_node]['pos']
            dist= math.dist(pos_current, pos_d)
            if dist < min_dist:
                min_dist= dist
                nearest_dwelling= d_node
        #3.规划返回休息点的路径
        nodes, edges= self.path_planning.astar_search(current, nearest_dwelling)
        if combined_nodes:
            combined_nodes.extend(nodes[1:])#跳过第一个节点，避免重复
        else:
            combined_nodes.extend(nodes)
        combined_edges.extend(edges)

        return (combined_nodes, combined_edges)
    


        
    def build_order_nodes(self, path_nodes: list, task: dict) -> list:
        """
        Task 7: Assign VDA 5050 actions to each node in the combined path.

        Action rules (matching the dwelling -> pick -> process -> drop -> dwelling structure):
            - For stations with actionType 'pick' or 'drop' (TRANSFER):
                * The node immediately BEFORE the station in path_nodes
                  gets an 'init_fine_positioning' action.
                * The station node itself gets a 'pick' or 'drop' action.
            - For stations with actionType 'process' (PROCESS):
                * The station node gets a 'process' action with
                  processingTime from station['processingTime'].
            - All other nodes (intermediate + dwelling): empty actions list.

        Use uuid.uuid4() to generate unique actionId strings.

        For each node, also look up:
            - x, y from self.graph.nodes[n]['pos']
            - theta from self.graph.nodes[n].get('vehicleTypeNodeProperties', [])
              for the entry matching vehicleTypeId 'Longitudinal_Conveyor'.
              If the value is None or the string "None", use theta=None.

        Returns a list of node dicts:
            [{"nodeId": n, "x": x, "y": y, "theta": theta_or_None, "actions": [...]}, ...]

        Hint: build a lookup dict {station_nodeId: station} first, then iterate
        through path_nodes and check each node against the lookup.
        To detect the 'before TRANSFER' node, look ahead:
            if i + 1 < len(path_nodes) and path_nodes[i+1] is a TRANSFER station:
                add init_fine_positioning to path_nodes[i]
        """
        nodes_result= []

        station_lookup= {s['nodeId']: s for s in task['stations']}
        for i, n_id in enumerate(path_nodes):
            actions= []
            if n_id in station_lookup:
                st = station_lookup[n_id]
                if st['actionType'] in ['pick', 'drop']:
                    actions.append({
                        "actionType": st['actionType'],
                        "actionId": str(uuid.uuid4()),
                        "blockingType": "HARD"
                    })
                elif st['actionType'] == 'process':
                    actions.append({
                        "actionType": "process",
                        "actionId": str(uuid.uuid4()),
                        "blockingType": "HARD",
                        "processingTime": st['processingTime']
                    })#如果是站点，添加相应的动作

            if i+1<len(path_nodes):
                next_node = path_nodes[i+1]
                if next_node in station_lookup:
                    next_st=station_lookup[next_node]
                    if next_st['actionType'] in ['pick', 'drop']:
                        actions.append({
                            "actionType": "init_fine_positioning",
                            "actionId": str(uuid.uuid4()),
                            "blockingType": "HARD"
                        })#如果下一个节点是转运站，当前节点需定位

            node_props=self.graph.nodes[n_id].get('vehicleTypeNodeProperties', [])
            theta= None
            if node_props:
                for prop in node_props:
                    if prop['vehicleTypeId'] == 'Longitudinal_Conveyor':
                        val=prop.get('theta')
                        if val is not None and val != "None":
                            theta= float(val)
            nodes_result.append({
                "nodeId": n_id,
                "x": self.graph.nodes[n_id]['pos'][0],
                "y": self.graph.nodes[n_id]['pos'][1],
                "theta": theta,
                "actions": actions
            })
        return nodes_result
    

    def build_order_edges(self, path_nodes: list, path_edges: list) -> list:
        """
        Task 7: Build the edges input list for generate_order_message().

        Returns:
            [{
              "edgeId": path_edges[i],
              "startNodeId": path_nodes[i],
              "endNodeId": path_nodes[i+1],
              "actions": []}]
        """
        edges_result= []
        for i,e_id in enumerate(path_edges):
            edges_result.append({
                "edgeId": e_id,
                "startNodeId": path_nodes[i],
                "endNodeId": path_nodes[i+1],
                "actions": []
            })
        return edges_result
    


class PathPlanning:
    """
    A* shortest-path search over the graph.

    Task 6: Implement all three methods.
    Graph interfaces you will likely need:
        self.graph.get_connected_nodes(node_id)  -> list of neighbour node IDs
        self.graph.get_connected_edge(a, b)       -> edge ID connecting a and b
        self.graph.nodes[node_id]['pos']           -> (x, y) position tuple
    """

    def __init__(self, config_data, graph) -> None:
        self.config_data = config_data
        self.graph = graph

    def astar_search(self, start_node: str, goal_node: str) -> tuple:
        """
        Find the shortest path from start_node to goal_node using A*.

        Returns:
            (path_nodes, path_edges)
            path_nodes : ordered list of node IDs,  e.g. ["N5", "N1", "N7", "N2"]
            path_edges : ordered list of edge IDs,  e.g. ["E21", "E1", "E2"]
            len(path_edges) == len(path_nodes) - 1

        Returns (None, None) if no path exists.

        Task 6 hints:
            - Handle the edge case start_node == goal_node first:
                  return ([start_node], [])
            - Use a min-heap (heapq) ordered by f = g + h:
                  import heapq
                  open_set = []
                  heapq.heappush(open_set, (f, node_id))
            - g(n) = accumulated travel cost from start to n.
                  Increment it with self.get_distance(current, neighbour).
            - h(n) = self.get_h(n, goal_node)  -- Euclidean, never overestimates.
            - Keep a came_from dict to reconstruct the path on success.
            - Build path_edges by calling self.graph.get_connected_edge(a, b)
              for each consecutive pair in the reconstructed path_nodes.
        """
        if start_node == goal_node:
            return ([start_node], [])#1.起点即终点情况
        
        #2.初始化
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {start_node: 0.0}

        while open_set:
            current_f, current_node = heapq.heappop(open_set)#3.取出f值最小的节点
            #4.找到终点，并回溯
            if current_node == goal_node:
                path_nodes = [current_node]
                while current_node in came_from:
                    current_node = came_from[current_node]
                    path_nodes.append(current_node)
                    #翻转列表
                path_nodes.reverse()
                #5.获取边
                path_edges = []
                for i in range(len(path_nodes) - 1):
                    edge = self.graph.get_connected_edge(path_nodes[i], path_nodes[i + 1])
                    path_edges.append(edge)
                return (path_nodes, path_edges)
            #6.遍历邻居节点
            for neighbour in self.graph.get_connected_nodes(current_node):
                tentative_g_score = g_score[current_node]+self.get_distance(current_node, neighbour)

                if neighbour not in g_score or tentative_g_score < g_score[neighbour]:
                    came_from[neighbour] = current_node
                    g_score[neighbour] = tentative_g_score
                    #f= g+h
                    f= tentative_g_score + self.get_h(neighbour, goal_node)
                    heapq.heappush(open_set, (f, neighbour))
        return (None, None)
                
                       
        

    def get_h(self, current_node: str, goal_node: str) -> float:
        """
        Heuristic -- Euclidean distance from current_node to goal_node.

        Task 6: Use self.graph.nodes[node_id]['pos'] for (x, y) coordinates,
        then return math.dist(pos_current, pos_goal).
        This heuristic is admissible (straight-line <= actual path length).
        """
        pos_current = self.graph.nodes[current_node]['pos']
        pos_goal = self.graph.nodes[goal_node]['pos']
        return math.dist(pos_current, pos_goal)


    def get_distance(self, start_node: str, goal_node: str) -> float:
        """
        Actual edge cost -- Euclidean distance between two adjacent nodes.

        Task 6: Same formula as get_h(); used as the g-score increment per step.
        """
        pos_start = self.graph.nodes[start_node]['pos']
        pos_goal = self.graph.nodes[goal_node]['pos']
        return math.dist(pos_start, pos_goal)