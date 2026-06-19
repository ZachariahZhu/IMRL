import json
with open('data/input_files/lif_file.json') as f:
    lif_data = json.load(f)
print(json.dumps(lif_data["layouts"][0]["vehicleTypes"], indent=4))
