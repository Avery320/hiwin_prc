# ========== Trajectory Loader (python01) ==========
# Inputs:
#   Path   (str)  - JSON file path (default: /Users/avery_tsai/project/hiwin_prc/config/output.json)
#   Reload (bool) - Force reload, clear cache (default: False)
# Outputs:
#   Data  (any)  - Raw JSON content (dict/list)
#   Text  (str)  - Raw JSON string (optional)
#   JSONL (str) - Converted JSONL text (one JSON row per line)
#   Lines (list) - JSONL lines as list of strings
#   OutPath (str) - Where JSONL was (would be) written on disk
#   Count (int) - Number of frames/lines parsed

#
# Notes:
# - Designed for Grasshopper GhPython. Paste the file content into a GhPython component
#   or use execfile() / import techniques as you prefer.

import os
import json

try:
    import scriptcontext as sc
except ImportError:
    sc = None

CACHE_KEY = 'GH_TRAJ_CACHE'


def _get_cache():
    if sc is None:
        return {}
    if not hasattr(sc, 'sticky'):
        return {}
    cache = sc.sticky.get(CACHE_KEY)
    if cache is None:
        cache = {}
        sc.sticky[CACHE_KEY] = cache
    return cache


def _read_json_obj(path):
    # Prefer UTF-8; fall back to UTF-8-sig if BOM exists
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except ValueError:
        with open(path, 'r', encoding='utf-8-sig') as f:
            return json.load(f)


def _read_text(path):
    # Read as text for debugging/inspection
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception:
        with open(path, 'r', encoding='utf-8-sig', errors='ignore') as f:
            return f.read()

# ===== Helpers for JSON -> JSONL (joint1..joint6) =====

def _normalize_name(name):
    try:
        return str(name).strip().lower().replace('_', '')
    except Exception:
        return ''


def _pick_value_by_name(names, positions, j_index):
    target = 'joint{}'.format(j_index)
    try:
        norm = [_normalize_name(n) for n in (names or [])]
        for i, n in enumerate(norm):
            if n == target:
                return float(positions[i]) if i < len(positions or []) else 0.0
    except Exception:
        pass
    # fallback by index (1-based)
    if isinstance(positions, (list, tuple)) and len(positions) >= j_index:
        return float(positions[j_index - 1])
    return 0.0


def _ensure_six(values):
    vals = list(values[:6]) if isinstance(values, (list, tuple)) else []
    if len(vals) < 6:
        vals += [0.0] * (6 - len(vals))
    return vals


def _parse_frames_any(obj):
    # Shape C: {"joint_names": [...], "points": [ {"positions": [...]}, ... ]}
    if isinstance(obj, dict) and isinstance(obj.get('points'), list):
        names = obj.get('joint_names') or obj.get('names') or []
        if not isinstance(names, (list, tuple)):
            names = []
        frames = []
        for pt in obj['points']:
            if not isinstance(pt, dict):
                continue
            positions = pt.get('positions') or pt.get('position') or []
            if not isinstance(positions, (list, tuple)):
                positions = []
            jvals = [
                _pick_value_by_name(names, positions, 1),
                _pick_value_by_name(names, positions, 2),
                _pick_value_by_name(names, positions, 3),
                _pick_value_by_name(names, positions, 4),
                _pick_value_by_name(names, positions, 5),
                _pick_value_by_name(names, positions, 6),
            ]
            jvals = _ensure_six(jvals)
            frames.append({
                'joint1': jvals[0],
                'joint2': jvals[1],
                'joint3': jvals[2],
                'joint4': jvals[3],
                'joint5': jvals[4],
                'joint6': jvals[5],
            })
        return frames

    # Shape A/B: joint_states list or plain list of entries
    seq = None
    if isinstance(obj, dict):
        if isinstance(obj.get('joint_states'), list):
            seq = obj['joint_states']
        else:
            # fallback: first list-like value
            for v in obj.values():
                if isinstance(v, list):
                    seq = v
                    break
    elif isinstance(obj, list):
        seq = obj

    if not isinstance(seq, list) or not seq:
        return []

    frames = []
    for entry in seq:
        if not isinstance(entry, dict):
            continue
        names = entry.get('name') or entry.get('names') or []
        positions = entry.get('position') or entry.get('positions') or []
        if not isinstance(names, (list, tuple)):
            names = []
        if not isinstance(positions, (list, tuple)):
            positions = []
        jvals = [
            _pick_value_by_name(names, positions, 1),
            _pick_value_by_name(names, positions, 2),
            _pick_value_by_name(names, positions, 3),
            _pick_value_by_name(names, positions, 4),
            _pick_value_by_name(names, positions, 5),
            _pick_value_by_name(names, positions, 6),
        ]
        jvals = _ensure_six(jvals)
        frames.append({
            'joint1': jvals[0],
            'joint2': jvals[1],
            'joint3': jvals[2],
            'joint4': jvals[3],
            'joint5': jvals[4],
            'joint6': jvals[5],
        })
    return frames


def _frames_to_jsonl_lines(frames):
    try:
        return [json.dumps(row, ensure_ascii=False) for row in frames]
    except Exception:
        return []


def _write_text_safe(path, text):
    try:
        with open(path, 'w') as f:
            f.write(text)
    except Exception:
        pass

        with open(path, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception:
        with open(path, 'r', encoding='utf-8-sig', errors='ignore') as f:
            return f.read()


def _extract_stamp(entry):
    # Try common stamp fields: time, stamp, header.stamp
    t = entry.get('time')
    if isinstance(t, (int, float)):
        return float(t)

    stamp = entry.get('stamp') or entry.get('header', {}).get('stamp')
    if isinstance(stamp, dict):
        secs = stamp.get('secs') or stamp.get('sec') or stamp.get('seconds') or 0
        nsecs = stamp.get('nsecs') or stamp.get('nanosec') or stamp.get('nanosecs') or 0
        try:
            return float(secs) + float(nsecs) * 1e-9
        except Exception:
            return None
    return None


def _parse_joint_states(obj):
    # Accept two shapes:
    # 1) {"joint_states": [ {"name": [...], "position": [...], ...}, ... ]}
    # 2) [ {"name": [...], "position": [...], ...}, ... ]
    seq = None
    if isinstance(obj, dict):
        if 'joint_states' in obj and isinstance(obj['joint_states'], list):
            seq = obj['joint_states']
        else:
            # allow dict of frames keyed by something
            # fallback to values if they look like frames
            vals = list(obj.values())
            if vals and isinstance(vals[0], list):
                seq = vals[0]
    if seq is None and isinstance(obj, list):
        seq = obj
    if not isinstance(seq, list):
        return [], [], [], 0

    if not seq:
        return [], [], [], 0

    first = seq[0]
    names = list(first.get('name') or first.get('names') or [])
    if not names:
        # try common joint names default
        # if absent, attempt to infer from URDF typical pattern
        names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']

    frames = []
    times = []

    for entry in seq:
        pos = entry.get('position') or entry.get('positions') or []
        if not pos or len(pos) < len(names):
            # pad/truncate to names length
            pos = (list(pos) + [0.0] * len(names))[:len(names)]
        frame = {names[i]: float(pos[i]) for i in range(len(names))}
        frames.append(frame)
        times.append(_extract_stamp(entry))

    return frames, names, times, len(frames)


# -------- Grasshopper entry point --------
try:
    default_path = r"/Users/avery_tsai/project/hiwin_prc/config/output.json"
    _path = Path if 'Path' in globals() and Path else default_path
    _reload = Reload if 'Reload' in globals() else False

    Data = None
    Text = ''
    JSONL = ''
    Lines = []
    OutPath = ''
    Count = 0

    if _path and os.path.exists(_path):
        cache = _get_cache()
        mtime = os.path.getmtime(_path)
        ext = os.path.splitext(_path)[1].lower()
        key = ('TRAJ_JSON_JSONL', os.path.abspath(_path), mtime, ext)

        if _reload and cache:
            cache.clear()

        if key in cache:
            Data, Text, JSONL, Lines, OutPath, Count = cache[key]
        else:
            abspath = os.path.abspath(_path)
            if ext == '.jsonl':
                Text = _read_text(_path)
                Lines = [ln for ln in Text.splitlines() if ln.strip()]
                JSONL = '\n'.join(Lines) + ('\n' if Lines else '')
                OutPath = abspath
                Count = len(Lines)
                Data = None
            else:
                # Treat as JSON and convert to JSONL
                Data = _read_json_obj(_path)
                Text = _read_text(_path)
                frames = _parse_frames_any(Data)
                Lines = _frames_to_jsonl_lines(frames)
                JSONL = '\n'.join(Lines) + ('\n' if Lines else '')
                Count = len(Lines)
                OutPath = os.path.splitext(abspath)[0] + '.jsonl'
                if JSONL:
                    _write_text_safe(OutPath, JSONL)
            cache[key] = (Data, Text, JSONL, Lines, OutPath, Count)

except Exception:
    Data, Text, JSONL, Lines, OutPath, Count = None, '', '', [], '', 0

