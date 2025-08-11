import os

def export_raw(data, file_path):
    """
    將資料原封不動地寫入指定路徑
    data: 任意類型
    file_path: 絕對路徑
    """
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(str(data))

def export_jsonl(data, file_path):
    """
    將 list of JSON 字串逐行寫入標準 jsonl 格式
    data: list[str]，每個元素都是 JSON 字串
    file_path: 絕對路徑
    """
    import os
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w", encoding="utf-8") as f:
        for line in data:
            f.write(line.rstrip("\n") + "\n")

# Grasshopper端：
# 輸入 data (list[str])、file_path (str)、button (bool)
# 直接呼叫 export_jsonl(data, file_path)
if button == True:
    try:
        data  # type: ignore[name-defined]
        file_path  # type: ignore[name-defined]
        export_jsonl(data, file_path)
        result = True
    except Exception as e:
        result = False
        error_msg = str(e)
