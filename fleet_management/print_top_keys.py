import json
with open('data/input_files/lif_file.json') as f:
    lif_data = json.load(f)
print(lif_data.keys())
