import tkinter as tk
import time
import threading
import json
import os

class ColorVisualizer:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Farbvisualisator - JSON & Thread Loop")
        self.root.geometry("600x200") 
        self.root.configure(bg="#1a1a1a")

        self.container = tk.Frame(self.root, bg="#1a1a1a")
        self.container.pack(fill=tk.BOTH, expand=True, padx=0, pady=0)

    def update_colors(self, color_vector):
        """Aktualisiert die Farbflächen im Fenster."""
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
                # Fallback für ungültige Farbcodes
                color_block = tk.Frame(self.container, bg="#000000", relief=tk.FLAT)
                color_block.grid(row=0, column=index, sticky="nsew", padx=2, pady=2)

    def start(self):
        self.root.mainloop()


# ==========================================
# HIER LÄUFT DIE PARALLELE LOOP
# ==========================================
def json_schleifen_programm(visualizer, dateipfad, sleep_zeit):
    """
    Liest die JSON-Datei aus und durchläuft die Farbvektoren
    in einer permanenten While-Schleife.
    """
    print(f"Hintergrund-Thread gestartet. Lese '{dateipfad}'...")
    
    while True:
        # Prüfen, ob die Datei überhaupt existiert
        if not os.path.exists(dateipfad):
            print(f"Datei {dateipfad} nicht gefunden. Warte...")
            time.sleep(2)
            continue

        try:
            # 1. JSON-Datei frisch einlesen (so werden auch Live-Änderungen an der Datei erkannt!)
            with open(dateipfad, 'r', encoding='utf-8') as f:
                farb_daten = json.load(f)
            
            # 2. Durch alle Farbvektoren in der Liste loopen
            for farb_vektor in farb_daten:
                if isinstance(farb_vektor, list): # Sicherstellen, dass es eine Liste ist
                    
                    # UI-Update sicher an den Hauptthread übergeben
                    visualizer.root.after(0, lambda v=farb_vektor: visualizer.update_colors(v))
                    
                    # Dein gewünschtes time.sleep() am Ende jedes Schritts
                    time.sleep(sleep_zeit)
                    
        except json.JSONDecodeError:
            print("Fehler beim Lesen der JSON-Datei (eventuell unvollständig geschrieben). Versuche es gleich erneut...")
            time.sleep(1)
        except Exception as e:
            print(f"Unerwarteter Fehler: {e}")
            time.sleep(2)


# ==========================================
# START
# ==========================================
if __name__ == "__main__":
    # Pfad zu deiner JSON-Datei
    JSON_DATEI = "finalColors.json"
    
    # Visualizer-Fenster erstellen
    app = ColorVisualizer()

    # Thread starten: Übergibt die App, den Dateipfad und die gewünschte sleep-Zeit (z.B. 2.0 Sekunden)
    hintergrund_thread = threading.Thread(
        target=json_schleifen_programm, 
        args=(app, JSON_DATEI, 2.0), 
        daemon=True
    )
    hintergrund_thread.start()

    # GUI starten (blockiert hier, bleibt aber voll bedienbar)
    app.start()