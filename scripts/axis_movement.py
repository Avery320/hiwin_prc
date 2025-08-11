import json

# joint1~joint6 由 Grasshopper 輸入
axis_data = {
    "motion_type": "axis",
    "joint1": float(j1),
    "joint2": float(j2),
    "joint3": float(j3),
    "joint4": float(j4),
    "joint5": float(j5),
    "joint6": float(j6)
}

# result 可輸出 dict 或 JSON 字串，依 GH 需求選擇
result = axis_data
# 若要輸出 JSON 字串，請取消註解下行
# result = json.dumps(axis_data, indent=4)
result = json.dumps(data)