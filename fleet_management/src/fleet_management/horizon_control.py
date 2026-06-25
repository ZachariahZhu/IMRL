import math
import uuid
import time
import threading
import heapq
import json
#from fleet_management.traffic_controller import TrafficController


import tkinter as tk
from tkinter import messagebox

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

class Horizon_control:
    def __init__(self) -> None:
        
        threading.Thread(target=self.intmain, daemon=True).start()







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

    def test():
        if True:
            with open("test.txt","w") as w:
                print("test",file=w)
        pass

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
    def create_color_visualizer( color_vector, zeitschritt_ms):
        """
        Visualisiert einen Farbvektor ohne Schrift.
        :param color_vector: Liste von Hex-Farbcodes
        :param zeitschritt_ms: Zeitdauer in Millisekunden, nach der das Layout skaliert/aktualisiert wird
        """
        root = tk.Tk()
        root.title("Farbvisualisator")
        
        # Standardgröße beim Start
        root.geometry("600x200") 
        root.configure(bg="#1a1a1a")

        # Container für die Farbflächen
        container = tk.Frame(root, bg="#1a1a1a")
        container.pack(fill=tk.BOTH, expand=True, padx=0, pady=0)

        num_colors = len(color_vector)

        def layout_anpassen():
            # Löscht alte Blöcke, falls vorhanden, für ein sauberes Update
            for widget in container.winfo_children():
                widget.destroy()

            # Grid-Konfiguration basierend auf dem Vektor
            for i in range(num_colors):
                container.columnconfigure(i, weight=1)
            container.rowconfigure(0, weight=1)

            # Farbblöcke zeichnen (ohne Text-Labels!)
            for index, color in enumerate(color_vector):
                try:
                    color_block = tk.Frame(container, bg=color, relief=tk.FLAT)
                    color_block.grid(row=0, column=index, sticky="nsew", padx=2, pady=2)
                except tk.TclError:
                    # Fallback für ungültige Farbcodes (wird schwarz dargestellt)
                    color_block = tk.Frame(container, bg="#000000", relief=tk.FLAT)
                    color_block.grid(row=0, column=index, sticky="nsew", padx=2, pady=2)

        # Die Skalierung/Das Zeichnen wird erst nach dem gewünschten Zeitschritt getriggert
        root.after(zeitschritt_ms, layout_anpassen)

        root.mainloop()

    def multiplyMatrixes(matrix1,matrix2):
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
    @staticmethod
    def vectorize(matrix):
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

    def intmain(self) -> None:
        while True:
            catN=mouseN=catE=mouseE=None
            if True:
                with open("cat001.json", "r", encoding="utf-8") as f:
                    catN = json.load(f)

                with open("mouse001.json", "r", encoding="utf-8") as f2:
                    mouseN = json.load(f2)


                with open("cat001e.json", "r", encoding="utf-8") as e:
                    catE = json.load(e)

                with open("mouse001e.json", "r", encoding="utf-8") as e2:
                    mouseE = json.load(e2) 
            with open("result3.txt","w") as inter:
                print("Hello",file=inter)
                print("catN",file=inter)
                print(catN,file=inter)
                print("mouseN",file=inter)
                print(mouseN, file=inter)
                print("catE",file=inter)
                print(catE,file=inter)
                print("mouseE",file=inter)
                print(mouseE, file=inter)

                print()
            time.sleep(0.5)
                    

    
