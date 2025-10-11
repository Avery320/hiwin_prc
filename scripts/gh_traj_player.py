# ========== Trajectory JSONL Player (python02) ==========
# Inputs:
#   Lines (list[str]) - JSONL lines, each line like {"joint1": ..., ..., "joint6": ...}
#   JSONL (str)       - Raw JSONL text (fallback if Lines not provided)
#   Path  (str)       - Path to a .jsonl file (fallback if neither Lines nor JSONL provided)
#   S     (float)     - Normalized slider [0..1] to select line (preferred)
#   Index (int)       - Legacy index (0-based); used if S not provided
# Outputs:
#   J     (list[float]) - [joint1..joint6] for the selected line
#   Count (int)         - Number of lines
#   Line  (str)         - The selected raw JSONL line
#   I     (int)         - The selected 0-based index

import os
import json


def _read_text(path):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception:
        try:
            with open(path, 'r', encoding='utf-8-sig', errors='ignore') as f:
                return f.read()
        except Exception:
            return ''


def _build_lines():
    # Prefer Lines input if present
    lines_val = globals().get('Lines', None)
    if isinstance(lines_val, (list, tuple)):
        return [str(x) for x in lines_val if str(x).strip()]
    # Fallback to JSONL text
    jsonl_val = globals().get('JSONL', None)
    if isinstance(jsonl_val, str) and jsonl_val:
        return [ln for ln in jsonl_val.splitlines() if ln.strip()]
    # Fallback to Path to read file
    path_val = globals().get('Path', None)
    if path_val:
        text = _read_text(path_val)
        if text:
            return [ln for ln in text.splitlines() if ln.strip()]
    return []


def _to_index(count):
    if count <= 0:
        return 0
    s_val = globals().get('S', None)
    if s_val is None:
        s_val = globals().get('Phase', None)
    if s_val is not None:
        try:
            s = float(s_val)
        except Exception:
            s = 0.0
        if s < 0.0:
            s = 0.0
        if s > 1.0:
            s = 1.0
        return int(round(s * (count - 1))) if count > 1 else 0
    i_val = globals().get('Index', None)
    if i_val is not None:
        try:
            i = int(i_val)
        except Exception:
            i = 0
        if i < 0:
            i = 0
        if i >= count:
            i = count - 1
        return i
    return 0


def _parse_joints_from_line(line):
    # Expect dict with joint1..joint6; provide a fallback for position arrays
    try:
        obj = json.loads(line)
    except Exception:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    if isinstance(obj, dict):
        # Primary: joint1..joint6
        keys = ['joint1','joint2','joint3','joint4','joint5','joint6']
        if all(k in obj for k in keys):
            try:
                vals = [float(obj.get(k, 0.0)) for k in keys]
                return (vals + [0.0] * 6)[:6]
            except Exception:
                pass
        # Fallback: positions/position list
        pos = obj.get('positions') or obj.get('position') or obj.get('J')
        if isinstance(pos, (list, tuple)):
            try:
                vals = [float(v) for v in pos]
                vals = (vals + [0.0] * 6)[:6]
                return vals
            except Exception:
                pass
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# -------- Grasshopper entry point --------
try:
    _lines = _build_lines()
    Count = len(_lines)

    if Count <= 0:
        J, Line, I = [], '', 0
    else:
        I = _to_index(Count)
        Line = _lines[I]
        J = _parse_joints_from_line(Line)

except Exception:
    J, Count, Line, I = [], 0, '', 0

