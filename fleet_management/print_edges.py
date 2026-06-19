import json
with open('data/input_files/lif_file.json') as f:
    lif_data = json.load(f)
for e in lif_data["layouts"][0]["edges"]:
    allowed = []
    if "vehicleTypeEdgeProperties" in e:
        for prop in e["vehicleTypeEdgeProperties"]:
            vehicle_type_id = prop.get("vehicleTypeId")
            if vehicle_type_id is not None:
                allowed.append(vehicle_type_id)
    print(f"Edge {e['edgeId']} ({e['startNodeId']} -> {e['endNodeId']}): {allowed}")
