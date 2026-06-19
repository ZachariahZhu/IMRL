import json
with open('data/input_files/lif_file.json') as f:
    lif_data = json.load(f)
for e in lif_data["layouts"][0]["edges"]:
    if e["edgeId"] == "E3":
        print(json.dumps(e, indent=4))
