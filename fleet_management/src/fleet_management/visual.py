import math
import uuid
import time
import threading
import heapq
import json
#from fleet_management.traffic_controller import TrafficController


import tkinter as tk
from tkinter import messagebox

class ColorVisualizer:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Farbvisualisator - Threaded Loop")
        self.root.geometry("600x200") 
        self.root.configure(bg="#1a1a1a")

        self.container = tk.Frame(self.root, bg="#1a1a1a")
        self.container.pack(fill=tk.BOTH, expand=True, padx=0, pady=0)

    def update_colors(self, color_vector):
        """Aktualisiert die Farbflächen im Fenster sicher aus dem Thread heraus."""
        # Alte Blöcke löschen
        for widget in self.container.winfo_children():
            widget.destroy()

        num_colors = len(color_vector)
        
        # Grid konfigurieren
        for i in range(max(num_colors, self.container.grid_size()[0])):
            self.container.columnconfigure(i, weight=1 if i < num_colors else 0)
        self.container.rowconfigure(0, weight=1)

        # Farbblöcke zeichnen
        for index, color in enumerate(color_vector):
            try:
                color_block = tk.Frame(self.container, bg=color, relief=tk.FLAT)
                color_block.grid(row=0, column=index, sticky="nsew", padx=2, pady=2)
            except tk.TclError:
                color_block = tk.Frame(self.container, bg="#000000", relief=tk.FLAT)
                color_block.grid(row=0, column=index, sticky="nsew", padx=2, pady=2)

    def start(self):
        # Öffnet das Fenster und hält es aktiv
        self.root.mainloop()
class RunIt:
    
    def __init__(self) -> None:

        threading.Thread(target=self.run, daemon=True).start()
        self.col = ColorVisualizer()

    def run(self) -> None:
        fC=None
        with open("finalColors.json", "r", encoding="utf-8") as f:
            fC = json.load(f)
        self.col.start()
                    

    
