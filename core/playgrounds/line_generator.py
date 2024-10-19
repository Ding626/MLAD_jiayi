import math
import json
import numpy as np

def generateStraightLine(x1,y1,x2,y2,resolution):
    length = math.sqrt((x1-x2)**2 + (y1-y2)**2)
    point_num = math.ceil(length / resolution)
    x_ps = list(np.arange(x1, x2, (x2-x1)/point_num)) if (x1 != x2) else [x1]*point_num
    x_ps.append(x2)
    y_ps = list(np.arange(y1, y2, (y2-y1)/point_num)) if (y1 != y2) else [y1]*point_num
    y_ps.append(y2)

    points = [[float(x),float(y)] for x, y in zip(x_ps, y_ps)]
    return points, length

def generate_lanes():
    single = {"geometry": {"type": "MultiLineString", "coordinates":[] }, "type": "Feature", "properties": {"right_id": 0, "left_id": 0, "rchg_vld": 0, "length": 77.0373160877, "father_id": "0", "behavior": "s", "child_id": "0", "lchg_vld": 0, "id": 101}}
    fw = open("points.json", "w")
    idx=1
    points, length = generateStraightLine(500, 0, 800, 0, 2.5)
    # points = points[::-1]
    single["geometry"]["coordinates"] = [points]
    single["properties"]["length"] = length
    single["properties"]["id"] = idx
    fw.write(json.dumps(single, indent=1))
    fw.close()

generate_lanes()