import json

if 'joints' not in locals() or len(joints) != 6:
    raise ValueError("輸入的 'joints' 必須是一個包含 6 個數值的 list。")

axis_data = {
    "motion_type": "axis",
    "joint1": float(joints[0]),
    "joint2": float(joints[1]),
    "joint3": float(joints[2]),
    "joint4": float(joints[3]),
    "joint5": float(joints[4]),
    "joint6": float(joints[5])
}
# result 可輸出 dict 或 JSON 字串，依 GH 需求選擇
result = axis_data
# 若要輸出 JSON 字串，請取消註解下行
# result = json.dumps(axis_data, indent=4)
axis_command = json.dumps(axis_data)