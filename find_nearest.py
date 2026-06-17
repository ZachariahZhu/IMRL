import json, math

with open('fleet_management/data/input_files/lif_file.json', 'r') as f:
    data = json.load(f)

target = (7.50, 0.94)
nearest = None
min_d = float('inf')

for n in data['layouts'][0]['nodes']:
    pos = (n['nodePosition']['x'], n['nodePosition']['y'])
    d = math.dist(pos, target)
    if d < min_d:
        min_d = d
        nearest = n['nodeId']

print(f"Nearest to {target} is {nearest} with distance {min_d}")
