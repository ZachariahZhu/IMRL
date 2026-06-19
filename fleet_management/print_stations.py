import json
with open('data/input_files/lif_file.json') as f:
    lif_data = json.load(f)
for s in lif_data["layouts"][0]["stations"]:
    print(f"Station {s['stationId']}: {s.get('stationDescription')} -> {s.get('interactionNodeIds')}")
