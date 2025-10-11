#!/usr/bin/env python3
"""
Convert ROS JointState JSON to JSONL with fields joint1..joint6.

- Input default:  ./config/output.json
- Output default: ./config/joints.jsonl

The script is conservative and keeps the numeric values as-is (typically radians).
It tries to parse common shapes:
  1) {"joint_states": [ {"name": [...], "position": [...], ...}, ... ]}
  2) [ {"name": [...], "position": [...], ...}, ... ]

If names are present, it maps values to joint1..joint6 by matching normalized
names like joint_1 / joint1. Otherwise it takes the first 6 positions in order.
Missing joints are padded with 0.0.
"""

import os
import sys
import json
import argparse
from typing import Any, Dict, List, Tuple

DEFAULT_IN  = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'output.json')
DEFAULT_OUT = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'jjjj.jsonl')


def read_json(path: str) -> Any:
    # UTF-8 first, then fallback to UTF-8-sig
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except ValueError:
        with open(path, 'r', encoding='utf-8-sig') as f:
            return json.load(f)


def normalize_name(name: str) -> str:
    # Lowercase and remove underscores; focus on patterns like joint_1 -> joint1
    if not isinstance(name, str):
        return ''
    return name.strip().lower().replace('_', '')


def pick_value_by_name(names: List[str], positions: List[float], j_index: int) -> float:
    """Try to pick value for joint{j_index} by matching normalized names.
    Fallback: if not found and positions has enough elements, use positions[j_index-1]; else 0.0
    """
    target = f'joint{j_index}'
    try:
        norm = [normalize_name(n) for n in names]
        for i, n in enumerate(norm):
            if n == target:
                return float(positions[i]) if i < len(positions) else 0.0
    except Exception:
        pass
    # Fallback by index (1-based -> 0-based)
    if len(positions) >= j_index:
        return float(positions[j_index - 1])
    return 0.0


def ensure_six(values: List[float]) -> List[float]:
    vals = list(values[:6])
    if len(vals) < 6:
        vals += [0.0] * (6 - len(vals))
    return vals


def parse_frames(obj: Any) -> Tuple[List[Dict[str, float]], int]:
    """Parse various joint trajectory JSON shapes into rows of joint1..joint6.

    Supported shapes:
      A) {"joint_states": [ {"name": [...], "position": [...]}, ... ]}
      B) [ {"name": [...], "position": [...]}, ... ]
      C) {"joint_names": [...], "points": [ {"positions": [...]}, ... ]}
    """
    frames: List[Dict[str, float]] = []

    # Shape C: joint_names + points
    if isinstance(obj, dict) and isinstance(obj.get('points'), list):
        names = obj.get('joint_names') or obj.get('names') or []
        names = list(names) if isinstance(names, (list, tuple)) else []
        for pt in obj['points']:
            if not isinstance(pt, dict):
                continue
            positions = pt.get('positions') or pt.get('position') or []
            positions = list(positions) if isinstance(positions, (list, tuple)) else []
            jvals = [
                pick_value_by_name(names, positions, 1),
                pick_value_by_name(names, positions, 2),
                pick_value_by_name(names, positions, 3),
                pick_value_by_name(names, positions, 4),
                pick_value_by_name(names, positions, 5),
                pick_value_by_name(names, positions, 6),
            ]
            jvals = ensure_six(jvals)
            frames.append({
                'joint1': jvals[0],
                'joint2': jvals[1],
                'joint3': jvals[2],
                'joint4': jvals[3],
                'joint5': jvals[4],
                'joint6': jvals[5],
            })
        return frames, len(frames)

    # Shapes A/B: joint_states list or plain list of entries
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
        return [], 0

    for entry in seq:
        if not isinstance(entry, dict):
            continue
        names = entry.get('name') or entry.get('names') or []
        positions = entry.get('position') or entry.get('positions') or []
        try:
            names = list(names) if isinstance(names, (list, tuple)) else []
            positions = list(positions) if isinstance(positions, (list, tuple)) else []
        except Exception:
            names = []
            positions = []

        jvals = [
            pick_value_by_name(names, positions, 1),
            pick_value_by_name(names, positions, 2),
            pick_value_by_name(names, positions, 3),
            pick_value_by_name(names, positions, 4),
            pick_value_by_name(names, positions, 5),
            pick_value_by_name(names, positions, 6),
        ]
        jvals = ensure_six(jvals)

        frames.append({
            'joint1': jvals[0],
            'joint2': jvals[1],
            'joint3': jvals[2],
            'joint4': jvals[3],
            'joint5': jvals[4],
            'joint6': jvals[5],
        })

    return frames, len(frames)


def write_jsonl(path: str, rows: List[Dict[str, float]]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False))
            f.write('\n')


def main(argv: List[str]) -> int:
    ap = argparse.ArgumentParser(description='Convert ROS JointState JSON to JSONL with joint1..joint6 fields')
    ap.add_argument('--input', '-i', default=DEFAULT_IN, help='Input JSON path (default: %(default)s)')
    ap.add_argument('--output', '-o', default=DEFAULT_OUT, help='Output JSONL path (default: %(default)s)')
    args = ap.parse_args(argv)

    if not os.path.exists(args.input):
        print(f'[ERROR] Input file not found: {args.input}')
        return 1

    try:
        obj = read_json(args.input)
    except Exception as e:
        print(f'[ERROR] Failed to read JSON: {e}')
        return 2

    frames, count = parse_frames(obj)
    if count == 0:
        print('[WARN] No frames parsed; writing empty file')

    try:
        write_jsonl(args.output, frames)
    except Exception as e:
        print(f'[ERROR] Failed to write JSONL: {e}')
        return 3

    print(f'[OK] Wrote {count} lines to {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

